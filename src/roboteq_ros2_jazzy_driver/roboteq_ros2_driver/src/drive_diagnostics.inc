// Included inside namespace Roboteq. Publishing stays on the executor thread.
void Roboteq::record_drive_serial(const std::string &line, bool tx, bool ok)
{
    const auto steady = std::chrono::steady_clock::now();
    const auto stamp = this->get_clock()->now();
    std::lock_guard<std::mutex> lock(drive_diagnostics_mutex_);
    if (!tx) {
        const auto equals = line.find('=');
        if (equals != std::string::npos) {
            const auto key = line.substr(0, equals);
            if (key == "FF" || key == "FS" || key == "FM" || key == "A") {
                drive_replies_[key] = {line.substr(equals + 1), steady};
            }
        }
    }
    if (drive_serial_events_.size() >= 1024) {
        drive_serial_events_.pop_front();
        ++drive_serial_dropped_;
    }
    drive_serial_events_.push_back({stamp, steady, line, tx, ok, ++drive_serial_sequence_});
}

void Roboteq::publish_drive_diagnostics()
{
    using diagnostic_msgs::msg::DiagnosticArray;
    using diagnostic_msgs::msg::DiagnosticStatus;
    using diagnostic_msgs::msg::KeyValue;
    const auto now = std::chrono::steady_clock::now();
    const auto age = [&now](const auto &at) {
        return at.time_since_epoch().count() == 0 ? std::string("unknown") :
            std::to_string(std::chrono::duration<double>(now - at).count());
    };
    const auto value = [](DiagnosticStatus &s, const std::string &key, const std::string &text) {
        KeyValue pair;
        pair.key = key;
        pair.value = text;
        s.values.push_back(pair);
    };
    DiagnosticStatus state;
    state.name = "roboteq/drive";
    state.hardware_id = port;
    state.level = DiagnosticStatus::OK;
    state.message = "Driver observations; serial write success is not controller ACK";
    value(state, "serial_open", controller.isOpen() ? "true" : "false");
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        value(state, "command_linear_x_mps", std::to_string(linear_x));
        value(state, "command_angular_z_radps", std::to_string(angular_z));
        value(state, "command_right_rpm", std::to_string(right_rpm_command));
        value(state, "command_left_rpm", std::to_string(left_rpm_command));
    }
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        value(state, "encoder_initialized", encoder_initialized_ ? "true" : "false");
        value(state, "encoder_right_raw", std::to_string(odom_encoder_right));
        value(state, "encoder_left_raw", std::to_string(odom_encoder_left));
        value(state, "encoder_age_sec", age(last_encoder_receive_time_));
        const bool fresh = encoder_initialized_ &&
            std::chrono::duration<double>(now - last_encoder_receive_time_).count() <= 0.3;
        value(state, "measured_velocity_valid", fresh && velocity_estimate_valid_ ? "true" : "false");
        value(state, "measured_right_rpm", std::to_string(estimated_wheel_rpm_right_));
        value(state, "measured_left_rpm", std::to_string(estimated_wheel_rpm_left_));
        if (!fresh) {
            state.level = DiagnosticStatus::WARN;
            state.message = "Encoder missing/stale; zero odometry does not prove stopped wheels";
        }
    }
    std::deque<DriveSerialEvent> events;
    {
        std::lock_guard<std::mutex> lock(drive_diagnostics_mutex_);
        value(state, "last_stop_request", !drive_stop_seen_ ? "unknown" :
              drive_stop_requested_ ? "true" : "false");
        value(state, "stop_request_age_sec", age(drive_last_stop_));
        value(state, "cmdvel_received_count", std::to_string(drive_cmdvel_count_));
        value(state, "cmdvel_age_sec", age(drive_last_cmdvel_));
        value(state, "serial_events_dropped", std::to_string(drive_serial_dropped_));
        for (const auto &key : {"FF", "FS", "FM", "A"}) {
            const auto it = drive_replies_.find(key);
            value(state, std::string(key) + "_raw", it == drive_replies_.end() ? "unknown" : it->second.first);
            value(state, std::string(key) + "_age_sec", it == drive_replies_.end() ? "unknown" : age(it->second.second));
            const bool fresh = it != drive_replies_.end() &&
                std::chrono::duration<double>(now - it->second.second).count() < 3.0;
            value(state, std::string(key) + "_fresh", fresh ? "true" : "false");
            if (!fresh && state.level == DiagnosticStatus::OK) {
                state.level = DiagnosticStatus::WARN;
                state.message = "Controller telemetry missing/stale; inspect raw serial replies";
            }
        }
        events.swap(drive_serial_events_);
    }
    if (!controller.isOpen()) {
        state.level = DiagnosticStatus::ERROR;
        state.message = "Serial port closed";
    }
    DiagnosticArray message;
    message.header.stamp = this->get_clock()->now();
    message.status.push_back(state);
    drive_diagnostics_pub_->publish(message);
    for (const auto &event : events) {
        DiagnosticArray packet;
        packet.header.stamp = event.stamp;
        DiagnosticStatus serial;
        serial.name = event.tx ? "roboteq/serial_tx" : "roboteq/serial_rx";
        serial.hardware_id = port;
        serial.level = event.ok ? DiagnosticStatus::OK : DiagnosticStatus::ERROR;
        serial.message = event.tx ? "Host serial write; not controller acknowledgement" : "Controller serial line";
        value(serial, "line", event.line);
        value(serial, "host_write_ok", event.tx ? (event.ok ? "true" : "false") : "not_applicable");
        value(serial, "sequence", std::to_string(event.sequence));
        value(serial, "steady_time_ns", std::to_string(
            std::chrono::duration_cast<std::chrono::nanoseconds>(event.steady.time_since_epoch()).count()));
        packet.status.push_back(serial);
        (event.tx ? drive_serial_tx_pub_ : drive_serial_rx_pub_)->publish(packet);
    }
    // Read-only queries, only while a recorder/observer subscribes. Do not
    // replace the encoder stream, open a second reader, clear faults, or enable motors.
    const bool observing = drive_diagnostics_pub_->get_subscription_count() > 0 ||
        drive_serial_rx_pub_->get_subscription_count() > 0 ||
        drive_serial_tx_pub_->get_subscription_count() > 0;
    if (observing && controller.isOpen() &&
        (drive_last_query_.time_since_epoch().count() == 0 ||
         std::chrono::duration<double>(now - drive_last_query_).count() >= 1.0)) {
        drive_last_query_ = now;
        safe_serial_write("?FF\r");
        safe_serial_write("?FS\r");
        safe_serial_write("?FM\r");
        safe_serial_write("?A\r");
    }
}
