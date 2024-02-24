use crate::usb_serial_wrapper::UsbSerialWrapper;
use crate::serialport_conversions::*;
use crate::{
    codes, negotiation, parser, subnegotiation, Command, Negotiation, Parser, Subnegotiation,
};
use std::sync::Arc;
use rusb::{DeviceHandle, GlobalContext};
use serialport::{ClearBuffer, FlowControl, SerialPort};
use std::io::{self, BufWriter, Read, Write};
use std::net::{TcpListener, TcpStream, ToSocketAddrs};

#[derive(Debug)]
pub enum Error {
    Parsing(parser::Error),
    SerialInit(serialport::Error),
    Serial(io::Error),
    Tcp(io::Error),
}

pub struct Rfc2217Server {
    port_reader: Box<UsbSerialWrapper>,
    port_writer: BufWriter<Box<UsbSerialWrapper>>,
    tcp_conn: TcpStream,
    tcp_writer: BufWriter<TcpStream>,
    tcp_answer_buf: [u8; subnegotiation::MAX_SIZE],
    parser: Parser,
    signature: Vec<u8>,
    suspended_flow_control: FlowControl,
    break_state: bool,
}

impl Rfc2217Server {
    pub fn new_from_usb<A: ToSocketAddrs>(device_handle: Arc<DeviceHandle<GlobalContext>>, in_endpoint_address: u8, out_endpoint_address: u8, tcp_addr: A) -> Result<Self, Error> {
        // usb
        let read_device_handle = device_handle.clone();
        let write_device_handle = device_handle.clone();
        
        // tcp
        let listener = TcpListener::bind(tcp_addr).map_err(Error::Tcp)?;
        log::info!("bounding; waiting for connection...");
        let (connection, _) = listener.accept().map_err(Error::Tcp)?;
        log::info!("accepted connection");
        connection.set_nonblocking(true).map_err(Error::Tcp)?;
        let cloned_connection = connection.try_clone().map_err(Error::Tcp)?;

        Ok(Rfc2217Server {
            parser: Parser::new(),
            port_reader: Box::new(UsbSerialWrapper::new_from_usb(read_device_handle, in_endpoint_address, out_endpoint_address)),
            port_writer: BufWriter::new(Box::new(UsbSerialWrapper::new_from_usb(write_device_handle, in_endpoint_address, out_endpoint_address))),
            tcp_conn: connection,
            tcp_writer: BufWriter::new(cloned_connection),
            tcp_answer_buf: [0; subnegotiation::MAX_SIZE],
            signature: Vec::new(),
            suspended_flow_control: FlowControl::None,
            break_state: false,
        })
    }

    pub fn run(&mut self) -> Result<(), Error> {
        // Read and handle the data from the TCP connection
        log::trace!("reading from tcp...");
        let mut tcp_data = [0; 256];
        match self.tcp_conn.read(&mut tcp_data) {
            Ok(bytes_read) => {
                if bytes_read > 0 {
                    let bytes = &tcp_data[..bytes_read];
                    log::debug!("tcp read {bytes_read} bytes ({bytes:02x?}");
                    self.process_tcp_data(bytes)?;
                }
            }
            Err(err) => match err.kind() {
                io::ErrorKind::WouldBlock => {}
                _ => {
                    log::error!("tcp read error err = {err}");
                    return Err(Error::Tcp(err))
                },
            },
        }

        // Read and handle the data from the serial port
        log::trace!("reading from serial port..");
        let mut port_data = [0; 256];
        match self.port_reader.read(&mut port_data) {
            Ok(bytes_read) => {
                log::trace!("serial port read {bytes_read}");

                if bytes_read > 0 {
                    let bytes = &port_data[..bytes_read];
                    log::debug!("serial port read {bytes_read} bytes ({bytes:02x?}");

                    // send from serial port to tcp
                    log::debug!("writing serial port data to tcp");
                    for &byte in &port_data[..bytes_read] {
                        // Escape all IAC bytes
                        self.tcp_writer.write_all(&[byte]).map_err(Error::Tcp)?;
                        if byte == codes::IAC {
                            self.tcp_writer.write_all(&[byte]).map_err(Error::Tcp)?;
                        }
                    }
                    log::debug!("wrote port data to tcp");
                }
            }
            Err(err) => match err.kind() {
                io::ErrorKind::TimedOut => {}
                _ => {
                    log::error!("port read error err = {err}");
                    return Err(Error::Serial(err))
                },
            },
        }

        // Flush the buffered data to be sent
        self.port_writer.flush().map_err(Error::Serial)?;
        self.tcp_writer.flush().map_err(Error::Tcp)?;

        Ok(())
    }

    fn process_tcp_data(&mut self, bytes: &[u8]) -> Result<(), Error> {
        for &byte in bytes {
            if let Some(event) = self.parser.process_byte(byte).map_err(Error::Parsing)? {
                log::debug!("process_tcp_data: byte = {byte:02x} event = {event:?}");
                let answer_size = self.process_event(event).map_err(Error::Serial)?;
                let answer = &self.tcp_answer_buf[..answer_size];
                log::debug!("process_tcp_data: byte = {byte:02x} event = {event:?} answer = {answer:02x?}");
                self.tcp_writer
                    .write_all(answer)
                    .map_err(Error::Tcp)?;
            }
        }
        Ok(())
    }

    fn process_event(&mut self, event: parser::Event) -> Result<usize, io::Error> {
        match event {
            parser::Event::Data(byte) => {
                log::info!("parser::Event::Data byte = {byte:02x}");
                self.port_writer.write_all(&[byte])?;
                Ok(0)
            }
            parser::Event::Command(command) => {
                log::info!("parser::Event::Command command = {command:02x?}");
                self.process_command(command)
            },
            parser::Event::Negotiation(negotiation) => {
                log::info!("parser::Event::Negotiation negotiation = {negotiation:02x?}");
                self.process_negotiation(negotiation)
            },
            parser::Event::Subnegotiation(subnegotiation) => {
                log::info!("parser::Event::Subnegotiation subnegotiation = {subnegotiation:02x?}");
                self.process_subnegotiation(subnegotiation)
            }
        }
    }

    fn process_command(&mut self, command: Command) -> Result<usize, io::Error> {
        log::info!("process_command: command = {command:?}");
        match command {
            _ => Ok(0),
        }
    }

    fn process_negotiation(&mut self, negotiation: Negotiation) -> Result<usize, io::Error> {
        log::info!("process_negotiation: negotiation = {negotiation:?}");
        match negotiation.get_answer() {
            Some(answer) => {
                answer.serialize(&mut self.tcp_answer_buf[..negotiation::SIZE]);
                Ok(negotiation::SIZE)
            }
            None => Ok(0),
        }
    }

    fn process_subnegotiation(
        &mut self,
        subnegotiation: Subnegotiation,
    ) -> Result<usize, io::Error> {
        log::info!("process_subnegotiation: subnegotiation = {subnegotiation:?}");
        let answer_opt = match subnegotiation {
            Subnegotiation::SetSignature { data, size } => {
                log::info!("SetSignature");
                // An empty signature constitutes a signature query
                if size == 0 {
                    let mut data = [0; subnegotiation::MAX_DATA_SIZE];
                    let size = self.signature.len() as u8;
                    data.copy_from_slice(&self.signature);
                    Some(Subnegotiation::SetSignature { data, size })
                } else {
                    self.signature.copy_from_slice(&data[..size as usize]);
                    Some(subnegotiation)
                }
            }

            Subnegotiation::SetBaudRate(val) => {
                log::info!("SetBaudRate");
                if val == 0 {
                    Some(Subnegotiation::SetBaudRate(self.port_reader.baud_rate()?))
                } else {
                    self.port_reader.set_baud_rate(val)?;
                    Some(subnegotiation)
                }
            }

            Subnegotiation::SetDataSize(val) => {
                log::info!("SetDataSize");
                match u8_to_data_bits(val) {
                    Some(data_bits) => {
                        self.port_reader.set_data_bits(data_bits)?;
                        Some(subnegotiation)
                    }
                    None => Some(Subnegotiation::SetDataSize(data_bits_to_u8(
                        self.port_reader.data_bits()?,
                    ))),
                }
            },

            Subnegotiation::SetParity(val) => {
                log::info!("SetParity");
                match u8_to_parity(val) {
                    Some(parity) => {
                        self.port_reader.set_parity(parity)?;
                        Some(subnegotiation)
                    }
                    None => Some(Subnegotiation::SetParity(parity_to_u8(self.port_reader.parity()?))),
                }
            },

            Subnegotiation::SetStopSize(val) => {
                log::info!("SetStopSize");
                match u8_to_stop_bits(val) {
                    Some(stop_bits) => {
                        self.port_reader.set_stop_bits(stop_bits)?;
                        Some(subnegotiation)
                    }
                    None => Some(Subnegotiation::SetStopSize(stop_bits_to_u8(
                        self.port_reader.stop_bits()?,
                    ))),
                }
            },

            Subnegotiation::SetControl(val) => {
                log::info!("SetControl");
                self.handle_set_control(val)?
            },

            Subnegotiation::FlowControlSuspend => {
                log::info!("FlowControlSuspend");
                self.suspended_flow_control = self.port_reader.flow_control()?;
                self.port_reader.set_flow_control(FlowControl::None)?;
                Some(subnegotiation)
            }

            Subnegotiation::FlowControlResume => {
                log::info!("FlowControlResume");
                self.port_reader.set_flow_control(self.suspended_flow_control)?;
                Some(subnegotiation)
            }

            Subnegotiation::PurgeData(val) => {
                log::info!("PurgeData");
                self.handle_purge_data(val)?
            },

            _ => None,
        };

        match answer_opt {
            Some(answer) => Ok(answer.serialize_server(&mut self.tcp_answer_buf)),
            None => Ok(0),
        }
    }

    fn handle_set_control(&mut self, val: u8) -> Result<Option<Subnegotiation>, io::Error> {
        log::info!("handle_set_control val = {val:02x}");
        match val {
            0 => Ok(Some(Subnegotiation::SetControl(flow_control_to_u8(
                self.port_reader.flow_control()?,
            )))),
            1 | 2 | 3 => {
                self.port_reader
                    .set_flow_control(u8_to_flow_control(val).unwrap())?;
                Ok(Some(Subnegotiation::SetControl(val)))
            }
            4 => match self.break_state {
                true => Ok(Some(Subnegotiation::SetControl(5))),
                false => Ok(Some(Subnegotiation::SetControl(6))),
            },
            5 => {
                self.port_reader.set_break()?;
                self.break_state = true;
                Ok(Some(Subnegotiation::SetControl(val)))
            }
            6 => {
                self.port_reader.clear_break()?;
                self.break_state = false;
                Ok(Some(Subnegotiation::SetControl(val)))
            }
            7 => match self.port_reader.read_data_set_ready()? {
                true => Ok(Some(Subnegotiation::SetControl(8))),
                false => Ok(Some(Subnegotiation::SetControl(9))),
            },
            8 => {
                self.port_reader.write_data_terminal_ready(true)?;
                Ok(Some(Subnegotiation::SetControl(val)))
            }
            9 => {
                self.port_reader.write_data_terminal_ready(false)?;
                Ok(Some(Subnegotiation::SetControl(val)))
            }
            10 => match self.port_reader.read_clear_to_send()? {
                true => Ok(Some(Subnegotiation::SetControl(11))),
                false => Ok(Some(Subnegotiation::SetControl(12))),
            },
            11 => {
                self.port_reader.write_request_to_send(true)?;
                Ok(Some(Subnegotiation::SetControl(val)))
            }
            12 => {
                self.port_reader.write_request_to_send(false)?;
                Ok(Some(Subnegotiation::SetControl(val)))
            }
            _ => Ok(None),
        }
    }

    fn handle_purge_data(&mut self, val: u8) -> Result<Option<Subnegotiation>, io::Error> {
        log::info!("handle_purge_data val = {val:02x}");
        match val {
            1 => {
                self.port_reader.clear(ClearBuffer::Input)?;
                Ok(Some(Subnegotiation::PurgeData(val)))
            }
            2 => {
                self.port_reader.clear(ClearBuffer::Output)?;
                Ok(Some(Subnegotiation::PurgeData(val)))
            }
            3 => {
                self.port_reader.clear(ClearBuffer::Input)?;
                self.port_reader.clear(ClearBuffer::Output)?;
                Ok(Some(Subnegotiation::PurgeData(val)))
            }
            _ => Ok(None),
        }
    }
}

impl Negotiation {
    fn get_answer(&self) -> Option<Negotiation> {
        match (self.intent, self.option) {
            (
                negotiation::Intent::Will,
                negotiation::Option::Binary
                | negotiation::Option::ComPort
                | negotiation::Option::SuppressGoAhead,
            ) => Some(Negotiation {
                intent: negotiation::Intent::Do,
                option: self.option,
            }),
            (
                negotiation::Intent::Do,
                negotiation::Option::Binary
                | negotiation::Option::ComPort
                | negotiation::Option::SuppressGoAhead,
            ) => None,
            (negotiation::Intent::Will, _) => Some(Negotiation {
                intent: negotiation::Intent::Dont,
                option: self.option,
            }),
            (negotiation::Intent::Do, _) => Some(Negotiation {
                intent: negotiation::Intent::Wont,
                option: self.option,
            }),
            _ => panic!(),
        }
    }
}
