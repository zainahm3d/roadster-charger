use shared::state::State;
use std::{
    collections::VecDeque,
    io::Read,
    sync::RwLock,
    time::{Duration, SystemTime},
};
use zerocopy::TryFromBytes;

type StampedPacket = (SystemTime, State);

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut queue: VecDeque<StampedPacket> = VecDeque::new();

    let rec = rerun::RecordingStreamBuilder::new("roadster").connect_grpc()?;

    let port_name = find_port().expect("ESP32 not found");
    println!("ESP32 @ {}", port_name);

    // Baud rate doesn't matter because this port is CDC
    let mut port = serialport::new(port_name, 1_000_000)
        .timeout(Duration::from_millis(2_000))
        .open()
        .expect("Failed to open port");

    let mut packet_buf: Vec<u8> = Vec::new();
    let mut last_rx_time = std::time::Instant::now();
    let mut packet_complete = false;

    // TODO: Error handling
    loop {
        let mut buf = [0u8; 1024];

        let bytes_available = port.bytes_to_read().unwrap();
        if bytes_available > 0 {
            port.read_exact(&mut buf[0..bytes_available as usize])
                .unwrap();

            packet_buf.extend_from_slice(&buf[0..bytes_available as usize]);

            last_rx_time = std::time::Instant::now();
            packet_complete = false;
        }

        // Wait for serial port to go idle to frame packets
        if std::time::Instant::now().duration_since(last_rx_time)
            >= std::time::Duration::from_millis(10)
            && !packet_complete
        {
            let packet_len = core::mem::size_of::<State>();

            if packet_buf.len() == packet_len {
                let packet_buf = &mut packet_buf[0..packet_len];
                let state = State::try_mut_from_bytes(packet_buf).unwrap();
                queue_packet(state, &mut queue);
            }

            packet_complete = true;
            packet_buf.clear();
        }

        // Log each packet 100ms late to prevent underflowing the queue
        if let Some((time, _state)) = queue.front()
            && SystemTime::now().duration_since(*time).unwrap() >= Duration::from_millis(100)
        {
            let (time, state) = queue.pop_front().unwrap();
            send_to_rerun(&state, time, &rec);
        }
    }
}

fn find_port() -> Option<String> {
    let ports = serialport::available_ports().expect("No ports found");
    for p in ports {
        if let serialport::SerialPortType::UsbPort(usb_port) = p.port_type
            && usb_port
                .manufacturer
                .is_some_and(|m| m.eq_ignore_ascii_case("espressif"))
        {
            return Some(p.port_name);
        }
    }
    None
}

// Adds a packet to queue, but adds it 10x with equally spaced timestamps to allow
// rerun to plot @ 100FPS
fn queue_packet(new_packet: &State, queue: &mut VecDeque<StampedPacket>) {
    static LAST_PACKET_TIME: RwLock<Option<SystemTime>> = RwLock::new(None);

    let new_packet_time = SystemTime::now(); // TODO: make monotonic

    if let Some(last) = *LAST_PACKET_TIME.read().unwrap() {
        let time_since_last = new_packet_time
            .duration_since(LAST_PACKET_TIME.read().unwrap().unwrap())
            .unwrap();
        let interval = time_since_last / 10;

        let timestamps: [SystemTime; 10] =
            std::array::from_fn(|i| last + (interval.mul_f32(i as f32)));

        for stamp in timestamps {
            queue.push_back((stamp, *new_packet));
        }
    }

    *LAST_PACKET_TIME.write().unwrap() = Some(new_packet_time);
}

// TODO: come up with something less repetetive
fn send_to_rerun(s: &State, time: SystemTime, r: &rerun::RecordingStream) {
    use rerun::Scalars;

    r.set_time("roadster", time);

    r.log("roadster/tick", &Scalars::single(s.tick as f64))
        .unwrap();

    r.log(
        "roadster/board_temp",
        &Scalars::single(s.board_temp_c as f64),
    )
    .unwrap();

    r.log("roadster/target_ma", &Scalars::single(s.target_ma as f64))
        .unwrap();

    r.log("roadster/duty", &Scalars::single(s.duty as f64))
        .unwrap();

    r.log("roadster/input_mv", &Scalars::single(s.input_mv as f64))
        .unwrap();

    r.log("roadster/input_ma", &Scalars::single(s.input_ma as f64))
        .unwrap();

    r.log("roadster/output_mv", &Scalars::single(s.output_mv as f64))
        .unwrap();

    r.log("roadster/output_ma", &Scalars::single(s.output_ma as f64))
        .unwrap();

    r.log("roadster/pdo_mv", &Scalars::single(s.pdo_mv as f64))
        .unwrap();

    r.log("roadster/pdo_ma", &Scalars::single(s.pdo_ma as f64))
        .unwrap();

    r.log(
        "roadster/current_pi/output",
        &Scalars::single(s.i_ctrl.output as f64),
    )
    .unwrap();

    r.log("roadster/current_pi/p", &Scalars::single(s.i_ctrl.p as f64))
        .unwrap();

    r.log("roadster/current_pi/i", &Scalars::single(s.i_ctrl.i as f64))
        .unwrap();

    r.log(
        "roadster/voltage_pi/output",
        &Scalars::single(s.v_ctrl.output as f64),
    )
    .unwrap();

    r.log("roadster/voltage_pi/p", &Scalars::single(s.v_ctrl.p as f64))
        .unwrap();

    r.log("roadster/voltage_pi/i", &Scalars::single(s.v_ctrl.i as f64))
        .unwrap();
}
