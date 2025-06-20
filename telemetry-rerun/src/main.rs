use std::{io::BufRead, time::Duration};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let rec = rerun::RecordingStreamBuilder::new("roadster").connect_grpc()?;

    let port_name = find_port().expect("ESP32 not found");
    println!("ESP32 @ {}", port_name);

    // Baudrate doesn't matter because this port is CDC
    let port = serialport::new(port_name, 1_000_000)
        .timeout(Duration::from_millis(1000))
        .open()
        .expect("Failed to open port");

    let mut reader = std::io::BufReader::new(port);

    loop {
        let mut line = String::new();
        reader.read_line(&mut line).unwrap();

        // do a very illegal thing- use a debug printed struct as a data source
        if line.contains("&s = State {") {
            let mut buf: Vec<u8> = std::vec![];
            reader.read_until(b'}', &mut buf).unwrap();
            let message = String::from_utf8(buf).unwrap();
            let message = message.strip_suffix('}').unwrap();

            log_status_message(message, &rec);
        } else {
            print!("{}", line.trim()); // pass through normal print statements
        }
    }
}

fn find_port() -> Option<String> {
    let ports = serialport::available_ports().expect("No ports found");
    for p in ports {
        if let serialport::SerialPortType::UsbPort(usb_port) = p.port_type {
            if usb_port
                .manufacturer
                .is_some_and(|m| m.eq_ignore_ascii_case("espressif"))
            {
                return Some(p.port_name);
            }
        }
    }
    None
}

// This parser is temporary- future work includes sending the bare struct from the charger
// instead of a stringified form like this. Readable info is good for debugging right now.
fn log_status_message(message: &str, rec: &rerun::RecordingStream) {
    for line in message.split(",\n") {
        if line.contains(":") {
            let line = line.trim();

            let pair: Vec<&str> = line.split(':').collect();
            assert!(pair.len() == 2);

            let key = pair[0].trim().to_lowercase();
            let value = pair[1].trim().to_lowercase();

            if key == "tick" {
                rec.set_time("roadster", std::time::SystemTime::now());
            }

            if key == "mode" {
                let value: String = value.parse().unwrap();
                rec.log(format!("roadster/{key}"), &rerun::TextLog::new(value))
                    .unwrap();
            } else {
                let value: f64 = value.parse().unwrap();
                rec.log(format!("roadster/{key}"), &rerun::Scalars::single(value))
                    .unwrap();
            }
        }
    }
}
