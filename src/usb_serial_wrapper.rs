use std::{sync::Arc, time::Duration};

use rusb::{DeviceHandle, GlobalContext};
use serialport::SerialPort;

pub struct UsbSerialWrapper {
    device_handle: Arc<DeviceHandle<GlobalContext>>,
    in_endpoint_address: u8,
    out_endpoint_address: u8
}

impl UsbSerialWrapper {
    pub fn new_from_usb(device_handle: Arc<DeviceHandle<GlobalContext>>, in_endpoint_address: u8, out_endpoint_address: u8) -> Self {
        UsbSerialWrapper { 
            device_handle,
            in_endpoint_address,
            out_endpoint_address
        }
    }
    
}

impl std::io::Read for UsbSerialWrapper {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        let timeout = Duration::from_millis(5);
        match self.device_handle.read_bulk(self.in_endpoint_address, buf, timeout) {
            Ok(bytes_read) => {
                Ok(bytes_read)
            },
            Err(err) => {
                match err {
                    rusb::Error::Io => todo!(),
                    rusb::Error::InvalidParam => todo!(),
                    rusb::Error::Access => todo!(),
                    rusb::Error::NoDevice => todo!(),
                    rusb::Error::NotFound => todo!(),
                    rusb::Error::Busy => todo!(),
                    rusb::Error::Timeout => {
                        Ok(0)
                    }
                    rusb::Error::Overflow => todo!(),
                    rusb::Error::Pipe => todo!(),
                    rusb::Error::Interrupted => todo!(),
                    rusb::Error::NoMem => todo!(),
                    rusb::Error::NotSupported => todo!(),
                    rusb::Error::BadDescriptor => todo!(),
                    rusb::Error::Other => todo!(),
                }
            }
        }
    }
}

impl std::io::Write for UsbSerialWrapper {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        let timeout = Duration::from_secs(1);
        match self.device_handle.write_bulk(self.out_endpoint_address, buf, timeout) {
            Ok(bytes_read) => {
                Ok(bytes_read)
            },
            Err(_) => todo!(),
        }
    }

    fn flush(&mut self) -> std::io::Result<()> {
        // no-op
        Ok(())
    }
}

impl SerialPort for UsbSerialWrapper {
    fn name(&self) -> Option<String> {
        todo!()
    }

    fn baud_rate(&self) -> serialport::Result<u32> {
        // no-op
        Ok(115_200)
    }

    fn data_bits(&self) -> serialport::Result<serialport::DataBits> {
        // no-op
        Ok(serialport::DataBits::Eight)
    }

    fn flow_control(&self) -> serialport::Result<serialport::FlowControl> {
        // no-op
        Ok(serialport::FlowControl::None)
    }

    fn parity(&self) -> serialport::Result<serialport::Parity> {
        // no-op
        Ok(serialport::Parity::None)
    }

    fn stop_bits(&self) -> serialport::Result<serialport::StopBits> {
        // no-op
        Ok(serialport::StopBits::One)
    }

    fn timeout(&self) -> core::time::Duration {
        todo!()
    }

    fn set_baud_rate(&mut self, _baud_rate: u32) -> serialport::Result<()> {
        // no-op
        Ok(())
    }

    fn set_data_bits(&mut self, _data_bits: serialport::DataBits) -> serialport::Result<()> {
        // no-op
        Ok(())
    }

    fn set_flow_control(&mut self, _flow_control: serialport::FlowControl) -> serialport::Result<()> {
        // no-op
        Ok(())
    }

    fn set_parity(&mut self, _parity: serialport::Parity) -> serialport::Result<()> {
        // no-op
        Ok(())
    }

    fn set_stop_bits(&mut self, _stop_bits: serialport::StopBits) -> serialport::Result<()> {
        // no-op
        Ok(())
    }

    fn set_timeout(&mut self, _timeout: core::time::Duration) -> serialport::Result<()> {
        todo!()
    }

    fn write_request_to_send(&mut self, _level: bool) -> serialport::Result<()> {
        // no-op
        Ok(())
    }

    fn write_data_terminal_ready(&mut self, _level: bool) -> serialport::Result<()> {
        // no-op
        Ok(())
    }

    fn read_clear_to_send(&mut self) -> serialport::Result<bool> {
        todo!()
    }

    fn read_data_set_ready(&mut self) -> serialport::Result<bool> {
        todo!()
    }

    fn read_ring_indicator(&mut self) -> serialport::Result<bool> {
        todo!()
    }

    fn read_carrier_detect(&mut self) -> serialport::Result<bool> {
        todo!()
    }

    fn bytes_to_read(&self) -> serialport::Result<u32> {
        todo!()
    }

    fn bytes_to_write(&self) -> serialport::Result<u32> {
        todo!()
    }

    fn clear(&self, _buffer_to_clear: serialport::ClearBuffer) -> serialport::Result<()> {
        // no-op
        Ok(())
    }

    fn try_clone(&self) -> serialport::Result<Box<dyn SerialPort>> {
        todo!()
    }

    fn set_break(&self) -> serialport::Result<()> {
        // no-op
        Ok(())
    }

    fn clear_break(&self) -> serialport::Result<()> {
        // no-op
        Ok(())
    }
}
