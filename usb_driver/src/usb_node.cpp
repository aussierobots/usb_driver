// Copyright 2026 Australian Robotics Supplies & Technology
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <unistd.h>
#include <deque>
#include <mutex>
#include <string>
#include <chrono>
#include <ctime>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "usb_driver/visibility_control.h"
#include "usb_driver/usb.hpp"
#include "usb_driver_srvs/srv/connect.hpp"
#include "usb_driver_srvs/srv/disconnect.hpp"
#include "usb_msgs/msg/usb_frame.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace usb_driver
{


enum FrameType
{
  frame_in,       // in from the gps device
  frame_out       // out to the gps device
};
struct usb_queue_frame_t
{
  rclcpp::Time ts;
  std::vector<uint8_t> buf;
  FrameType frame_type;
};

class UsbNode : public rclcpp::Node
{
public:
  USB_DRIVER_PUBLIC
  explicit UsbNode(const rclcpp::NodeOptions & options)
  : Node("usb_node", rclcpp::NodeOptions()
      .automatically_declare_parameters_from_overrides(true)
      .use_intra_process_comms(true))
  {
    is_initialising_ = true;

    auto domain_id = options.context()->get_domain_id();  // ensure context is initialised
    RCLCPP_INFO(get_logger(), "starting %s with ROS_DOMAIN_ID=%ld", get_name(), domain_id);

    // used to indicate if threads and timers should shut down
    keep_running_ = true;

    this->get_node_base_interface()->get_context()->on_shutdown(
      [this]() {
        RCLCPP_INFO(this->get_logger(), "Initiating shutdown ...");
        this->keep_running_ = false;
        if (usbc_ != nullptr) {
          usbc_->shutdown();
        }
      });

    callback_group_usb_timer_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_usb_events_timer_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);


    device_readiness_state_ = DeviceReadinessState::UNREADY;
    has_been_connected_before_ = false;
    device_attached_ = false;

    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
    while (!parameters_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          get_logger(), "Interrupted while waiting for parameter client service. Exiting.");
        rclcpp::shutdown();
      }
      RCLCPP_WARN(get_logger(), "parameter client service not available, waiting again...");
    }

    check_for_frame_id_param(parameters_client);
    check_for_device_serial_param(parameters_client);
    check_for_vendor_id_param(parameters_client);
    check_for_product_id_param(parameters_client);

    auto qos = rclcpp::SystemDefaultsQoS();
    rclcpp::PublisherOptions pub_options;
    pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

    usb_frame_pub_ = create_publisher<usb_msgs::msg::USBFrame>(
      "usb_frame", qos, pub_options);

    // ros2 parameter call backs
    parameters_callback_handle_ =
      this->add_on_set_parameters_callback(
      std::bind( &UsbNode::on_set_parameters_callback,
        this, _1));

    std::string node_name(this->get_name());
    connect_service_ = this->create_service<usb_driver_srvs::srv::Connect>(
      node_name + "/connect", std::bind(&UsbNode::connect_callback, this, _1, _2));
    disconnect_service_ = this->create_service<usb_driver_srvs::srv::Disconnect>(
      node_name + "/disconnect", std::bind(&UsbNode::disconnect_callback, this, _1, _2));

    usb::connection_in_cb_fn connection_in_callback = std::bind(
      &UsbNode::usb_in_callback,
      this, _1);
    usb::connection_out_cb_fn connection_out_callback = std::bind(
      &UsbNode::usb_out_callback, this, _1);
    usb::connection_exception_cb_fn connection_exception_callback = std::bind(
      &UsbNode::usb_exception_callback, this, _1, _2);
    usb::hotplug_attach_cb_fn usb_hotplug_attach_callback = std::bind(
      &UsbNode::hotplug_attach_callback, this);
    usb::hotplug_detach_cb_fn usb_hotplug_detach_callback = std::bind(
      &UsbNode::hotplug_detach_callback, this);
    usb::connection_debug_cb_fn connection_debug_callback = std::bind(
      &UsbNode::usb_debug_callback, this, _1);

    usbc_ = std::make_shared<usb::Connection>(
      vendor_id_, product_id_,
      serial_str_);

    RCLCPP_DEBUG(get_logger(), "setting up usb callbacks ...");
    usbc_->set_in_callback(connection_in_callback);
    usbc_->set_out_callback(connection_out_callback);
    usbc_->set_exception_callback(connection_exception_callback);
    usbc_->set_hotplug_attach_callback(usb_hotplug_attach_callback);
    usbc_->set_hotplug_detach_callback(usb_hotplug_detach_callback);
    usbc_->set_debug_callback(connection_debug_callback);

    // initialise usb timer - once intialised the timer will be cancelled
    RCLCPP_DEBUG(get_logger(), "creating usb_init_timer_ ...");
    usb_init_timer_ = create_wall_timer(
      10ms, std::bind(&UsbNode::handle_usb_init_callback, this));

    RCLCPP_DEBUG(get_logger(), "creating usb_timer_ ...");
    usb_queue_.clear();
    usb_timer_ = create_wall_timer(
      10ms, std::bind(&UsbNode::usb_timer_callback, this),
      callback_group_usb_timer_);

    RCLCPP_DEBUG(get_logger(), "creating handle_usb_events_timer_ ...");
    handle_usb_events_timer_ = create_wall_timer(
      10ms, std::bind(&UsbNode::handle_usb_events_callback, this),
      callback_group_usb_events_timer_);

    is_initialising_ = false;
  }

  USB_DRIVER_LOCAL
  ~UsbNode()
  {
    keep_running_ = false;
    if (usbc_) {
      usbc_->shutdown();
      usbc_.reset();
    }
    RCLCPP_INFO(this->get_logger(), "finished");
  }

private:
  bool keep_running_ = true;
  bool device_attached_ = false;
  bool is_initialising_;
  enum class DeviceReadinessState
  {
    UNREADY,    // Device not operational
    READY       // Device operational
  };
  DeviceReadinessState device_readiness_state_ = DeviceReadinessState::UNREADY;
  bool has_been_connected_before_ = false;  // Track if this is reconnection

  rclcpp::CallbackGroup::SharedPtr callback_group_usb_timer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_usb_events_timer_;

  std::shared_ptr<usb::Connection> usbc_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;

// specific to libusb to to process events asynchronously
  rclcpp::TimerBase::SharedPtr handle_usb_events_timer_;

// don't want to block fetching of messages from the ublox device,
// so put them in a queue, with a timestamp to be processed later
  std::deque<usb_queue_frame_t> usb_queue_;
  std::mutex usb_queue_mutex_;

  rclcpp::TimerBase::SharedPtr usb_timer_;

// once the usb is initialised this timer is disabled
  rclcpp::TimerBase::SharedPtr usb_init_timer_;

  std::string frame_id_;
  const std::string FRAME_ID_PARAM_NAME = "FRAME_ID";

  std::uint16_t vendor_id_;
  const std::string VENDOR_ID_PARAM_NAME = "VENDOR_ID";

  std::uint16_t product_id_;
  const std::string PRODUCT_ID_PARAM_NAME = "PRODUCT_ID";

  std::string serial_str_;
  const std::string DEV_STRING_PARAM_NAME = "DEVICE_SERIAL_STRING";

  rclcpp::Publisher<usb_msgs::msg::USBFrame>::SharedPtr usb_frame_pub_;


  rclcpp::Service<usb_driver_srvs::srv::Connect>::SharedPtr connect_service_;
  rclcpp::Service<usb_driver_srvs::srv::Disconnect>::SharedPtr disconnect_service_;

  USB_DRIVER_LOCAL
  void log_usbc()
  {
    if (!usbc_) {
      RCLCPP_WARN(this->get_logger(), "USB connection object is null");
      return;
    }

    // Build endpoint string dynamically to avoid printing invalid 0x00 for ep_comms
    std::ostringstream ep_info;
    ep_info << "ep_data out: 0x" << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(usbc_->ep_data_out_addr())
            << " in: 0x" << std::setw(2) << static_cast<int>(usbc_->ep_data_in_addr());
    if (usbc_->ep_comms_in_addr() != 0) {
      ep_info << " ep_comms in: 0x" << std::setw(2) << static_cast<int>(usbc_->ep_comms_in_addr());
    }

    RCLCPP_INFO(
      this->get_logger(),
      "usb vendor_id: 0x%04x product_id: 0x%04x "
      "serial_str: %s bus: %03d address: %03d "
      "port_number: %d speed: %s num_interfaces: %u %s",
      usbc_->vendor_id(),
      usbc_->product_id(),
      usbc_->serial_str().c_str(),
      usbc_->bus_number(),
      usbc_->device_address(),
      usbc_->port_number(),
      usbc_->device_speed_txt(),
      usbc_->num_interfaces(),
      ep_info.str().c_str());
  }

  USB_DRIVER_LOCAL
  void check_for_vendor_id_param(rclcpp::SyncParametersClient::SharedPtr param_client)
  {
    // TODO - default to 0 for now, but should be set to the vendor id of the device
    vendor_id_ = 0;

    // Check if the parameter exists
    if (!param_client->has_parameter(VENDOR_ID_PARAM_NAME)) {
        RCLCPP_INFO(
          this->get_logger(), "Parameter %s not found.",
          VENDOR_ID_PARAM_NAME.c_str());
      return;
    }

    // Get the parameter value
    vendor_id_ = param_client->get_parameter<std::uint16_t>(VENDOR_ID_PARAM_NAME);
    RCLCPP_INFO(
      this->get_logger(), "Parameter %s found with value: %s",
      VENDOR_ID_PARAM_NAME.c_str(), std::to_string(vendor_id_).c_str());
  }

  USB_DRIVER_LOCAL
  void check_for_product_id_param(rclcpp::SyncParametersClient::SharedPtr param_client)
  {
    product_id_= 0;  // default to 0 for now, but should be set to the product id of the device

    // Check if the parameter exists
    if (!param_client->has_parameter(PRODUCT_ID_PARAM_NAME)) {
        RCLCPP_INFO(
          this->get_logger(), "Parameter %s not found.",
          PRODUCT_ID_PARAM_NAME.c_str());
      return;
    }

    // Get the parameter value
    product_id_ = param_client->get_parameter<std::uint16_t>(PRODUCT_ID_PARAM_NAME);
    RCLCPP_INFO(
      this->get_logger(), "Parameter %s found with value: %s",
      PRODUCT_ID_PARAM_NAME.c_str(), std::to_string(product_id_).c_str());
  }

  USB_DRIVER_LOCAL
  void check_for_device_serial_param(rclcpp::SyncParametersClient::SharedPtr param_client)
  {
    // default to empty string
    serial_str_ = "";

    // Check if the parameter exists
    if (!param_client->has_parameter(DEV_STRING_PARAM_NAME)) {
        RCLCPP_INFO(
          this->get_logger(), "Parameter %s not found, will use first device.",
          DEV_STRING_PARAM_NAME.c_str());
      return;
    }

    // Get the parameter value
    serial_str_ = param_client->get_parameter<std::string>(DEV_STRING_PARAM_NAME);
    RCLCPP_INFO(
      this->get_logger(), "Parameter %s found with value: %s",
      DEV_STRING_PARAM_NAME.c_str(), serial_str_.c_str());
  }

  USB_DRIVER_LOCAL
  void check_for_frame_id_param(rclcpp::SyncParametersClient::SharedPtr param_client)
  {
    // default to usb frame_id
    frame_id_ = "usb";
    // Check if the parameter exists
    if (!param_client->has_parameter(FRAME_ID_PARAM_NAME)) {
      RCLCPP_INFO(
        this->get_logger(), "Parameter %s not found, defaulting to 'usb' frame_id",
        FRAME_ID_PARAM_NAME.c_str());
      return;
    }

    // Get the parameter value
    frame_id_ = param_client->get_parameter<std::string>(FRAME_ID_PARAM_NAME);
    RCLCPP_INFO(
      this->get_logger(), "Parameter %s found with value: %s",
      FRAME_ID_PARAM_NAME.c_str(), frame_id_.c_str());
  }

  // on set parameters callback function
  USB_DRIVER_LOCAL
  rcl_interfaces::msg::SetParametersResult on_set_parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    RCLCPP_DEBUG(
      get_logger(), "starting on_set_parameters_callback with %ld parameters",
      parameters.size());

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    // TODO - check if the parameters are valid and set result.successful to false if not

    if (result.successful) {
      RCLCPP_DEBUG(get_logger(), "finished on_set_parameters_callback - result successful");
    } else {
      RCLCPP_WARN(
        get_logger(), "finished on_set_parameters_callback - result successful: %d reason: %s",
        result.successful, result.reason.c_str());
    }

    return result;
  }


  USB_DRIVER_LOCAL
  void connect_callback (
    const std::shared_ptr<usb_driver_srvs::srv::Connect::Request> request,
    std::shared_ptr<usb_driver_srvs::srv::Connect::Response> response)
  {
    (void)request;
    RCLCPP_WARN(
      get_logger(), "connect service called");

    (void)response;
  }

  USB_DRIVER_LOCAL
  void disconnect_callback (
    const std::shared_ptr<usb_driver_srvs::srv::Disconnect::Request> request,
    std::shared_ptr<usb_driver_srvs::srv::Disconnect::Response> response)
  {
    (void)request;
    RCLCPP_WARN(
      get_logger(), "connect service called");

    (void)response;
  }

  USB_DRIVER_LOCAL
  void perform_usb_initialization(bool from_timer = false)
  {
    if (from_timer) {
      RCLCPP_INFO(get_logger(), "Starting USB initialization");
    } else {
      RCLCPP_INFO(get_logger(), "Starting USB initialization (from hotplug)");
    }

    try {
      RCLCPP_DEBUG(get_logger(), "usbc->init() ...");
      usbc_->init();

      // Check device readiness
      if (!usbc_->devh_valid()) {
        RCLCPP_DEBUG(get_logger(), "Device not ready for init_async");
        return;
      }

      // USB async init
      RCLCPP_DEBUG(get_logger(), "usbc->init_async() ...");
      usbc_->init_async();

      log_usbc();

      RCLCPP_INFO(get_logger(), "USB initialization completed successfully");

      // Disable timer once connected and initialized
      if (usb_init_timer_) {
        usb_init_timer_->cancel();
        RCLCPP_DEBUG(get_logger(), "USB initialization complete - timer disabled");
      }
    } catch (std::string const & msg) {
      RCLCPP_ERROR(this->get_logger(), "usb init error: %s", msg.c_str());
      if (usbc_ != nullptr) {
        usbc_->shutdown();
        usbc_.reset();
      }
      rclcpp::shutdown();
    } catch (usb::UsbException & e) {
      RCLCPP_ERROR(this->get_logger(), "usb init UsbException: %s", e.what());
      if (usbc_ != nullptr) {
        usbc_->shutdown();
        usbc_.reset();
      }
      rclcpp::shutdown();
    } catch (std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "usb init events exception: %s", e.what());
      if (usbc_ != nullptr) {
        usbc_->shutdown();
        usbc_.reset();
      }
      rclcpp::shutdown();
    } catch (const char * msg) {
      RCLCPP_ERROR(this->get_logger(), "usb init events - %s", msg);
      if (usbc_ != nullptr) {
        usbc_->shutdown();
        usbc_.reset();
      }
      rclcpp::shutdown();
    }
  }

  USB_DRIVER_LOCAL
  void handle_usb_init_callback()
  {
    if (usbc_) {
      perform_usb_initialization(true);  // Let it handle device detection internally
    } else {
      RCLCPP_ERROR_ONCE(get_logger(), "usbc_ wasnt created when attempting to initialise USB!");
    }
  }

  // handle host in from usb device to host callback
  USB_DRIVER_LOCAL
  void usb_in_callback(libusb_transfer * transfer_in)
  {
    RCLCPP_DEBUG_ONCE(get_logger(), "initial usb_in_callback from usb ..");

    rclcpp::Time ts = rclcpp::Clock().now();

    size_t len = transfer_in->actual_length;
    unsigned char * buf = transfer_in->buffer;

    if (len > 0) {
      // queue the data so we can publish it later
      usb_queue_frame_t queue_frame {
        ts, std::vector<uint8_t>(buf, buf + len), FrameType::frame_in
      };
      {
        const std::lock_guard<std::mutex> lock(usb_queue_mutex_);
        usb_queue_.push_back(queue_frame);
      }

      // debug output
      std::ostringstream os;
      os << "0x";
      for (size_t i = 0; i < len; i++) {
        os << std::setfill('0') << std::setw(2) << std::right << std::hex << +buf[i];
      }

      RCLCPP_DEBUG(get_logger(), "in - buf: %s", os.str().c_str());
    } else {
      RCLCPP_DEBUG(get_logger(), "in - buf len is zero");
    }

    size_t num_transfer_in_queued = usbc_->queued_transfer_in_num();
    if (num_transfer_in_queued > 1) {
      RCLCPP_WARN(
        get_logger(), "too many transfer in transfers are queued (%lu)", num_transfer_in_queued);
    }
  }

// handle out to usb device to host callback
  USB_DRIVER_LOCAL
  void usb_out_callback(libusb_transfer * transfer_out)
  {
    RCLCPP_DEBUG_ONCE(get_logger(), "initial usb_out_callback from usb ..");

    rclcpp::Time ts = rclcpp::Clock().now();

    size_t len = transfer_out->actual_length;
    unsigned char * buf = transfer_out->buffer;

    usb_queue_frame_t queue_frame {
      ts, std::vector<uint8_t>(buf, buf + len), FrameType::frame_out
    };
    {
      const std::lock_guard<std::mutex> lock(usb_queue_mutex_);
      usb_queue_.push_back(queue_frame);
    }


    std::ostringstream os;
    os << "0x";
    for (int i = 0; i < transfer_out->actual_length; i++) {
      os << std::setfill('0') << std::setw(2) << std::right << std::hex << +buf[i];
    }

    RCLCPP_DEBUG(
      this->get_logger(), "out - status: %d length: %d buf: %s",
      transfer_out->status, transfer_out->actual_length,
      os.str().c_str());
  }

  USB_DRIVER_LOCAL
  void usb_exception_callback(usb::UsbException e, void * user_data)
  {
    (void)user_data;
    RCLCPP_ERROR(this->get_logger(), "ublox exception: %s", e.what());
  }

  USB_DRIVER_LOCAL
  void usb_debug_callback(std::string msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "usb: %s", msg.c_str());
  }

  USB_DRIVER_LOCAL
  void hotplug_attach_callback()
  {
    device_attached_ = true;

    bool is_reconnection = has_been_connected_before_;

    if (is_reconnection) {
      RCLCPP_INFO(
        get_logger(),
        "Device reconnected - restoring user parameters and refreshing device state");
      // USB async init

      perform_usb_initialization();  // Existing full init

      device_readiness_state_ = DeviceReadinessState::READY;
      RCLCPP_INFO(get_logger(), "Hotplug device re-connection completed");
    } else {
      // Initial connection: Full initialization
      device_readiness_state_ = DeviceReadinessState::READY;
      has_been_connected_before_ = true;
      RCLCPP_INFO(get_logger(), "Initial device connection completed");
    }
  }

  USB_DRIVER_LOCAL
  void hotplug_detach_callback()
  {
    RCLCPP_WARN(get_logger(), "USB device disconnected");
    device_attached_ = false;
    device_readiness_state_ = DeviceReadinessState::UNREADY;
  }

  USB_DRIVER_LOCAL
  void usb_queue_frame_in(usb_queue_frame_t * f)
  {
    if (f == nullptr) {
      RCLCPP_ERROR(get_logger(), "usb_queue_frame_in: frame pointer is null");
      return;
    }

    auto usb_frame = usb_msgs::msg::USBFrame();
    usb_frame.header.stamp = f->ts;
    usb_frame.header.frame_id = frame_id_;
    usb_frame.data = std::vector<uint8_t>(f->buf.data(), f->buf.data() + f->buf.size());
    usb_frame.frame_xfer_type = usb_msgs::msg::USBFrame::FRAME_XFER_TYPE_IN;

    usb_frame_pub_->publish(std::move(usb_frame));

    //
    // {
    //   const char * remove_any_of = "\n\r";
    //   std::string frame_data (reinterpret_cast<const char*>(f->buf.data()), f->buf.size());
    //   frame_data.erase(frame_data.find_last_not_of(remove_any_of) + 1);
    //   RCLCPP_DEBUG(get_logger(), "published usb frame data: %s", frame_data.c_str());
    // }
    // alteratively use ros2 topic echo /usb_frame --raw
    RCLCPP_DEBUG(get_logger(), "published usb frame in with %lu bytes", f->buf.size());
  }

  USB_DRIVER_LOCAL
  void usb_queue_frame_out(usb_queue_frame_t * f)
  {
    if (f == nullptr) {
      RCLCPP_ERROR(get_logger(), "usb_queue_frame_out: frame pointer is null");
      return;
    }

    auto usb_frame = usb_msgs::msg::USBFrame();
    usb_frame.header.stamp = f->ts;
    usb_frame.header.frame_id = frame_id_;
    usb_frame.data = std::vector<uint8_t>(f->buf.data(), f->buf.data() + f->buf.size());
    usb_frame.frame_xfer_type = usb_msgs::msg::USBFrame::FRAME_XFER_TYPE_OUT;

    usb_frame_pub_->publish(std::move(usb_frame));

    RCLCPP_DEBUG(get_logger(), "published usb frame out with %lu bytes", f->buf.size());
  }

  USB_DRIVER_LOCAL
  void usb_timer_callback()
  {
    if (!keep_running_) {
      RCLCPP_WARN(get_logger(), "shutting down handling of usb events ...");
      usb_timer_->cancel();
      return;
    }

    RCLCPP_DEBUG_ONCE(get_logger(), "initial usb_timer_callback ..");

    // if we dont have anything to do just return
    if (usb_queue_.size() == 0) {
      return;
    }

    while (usb_queue_.size() > 0) {
      try {
        usb_queue_frame_t f = usb_queue_[0];
        switch (f.frame_type) {
          case FrameType::frame_in:
            usb_queue_frame_in(&f);
            break;
          case FrameType::frame_out:
            usb_queue_frame_out(&f);
            break;
          default:
            RCLCPP_ERROR(
              get_logger(), "Unknown usb_queue frame_type: %d - doing nothing", f.frame_type);
        }
      } catch (const usb::UsbException & e) {
        RCLCPP_ERROR(get_logger(), "usb_queue_frame_in UsbException: %s", e.what());
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "usb_queue_frame_in exception: %s", e.what());
      }

      {
        const std::lock_guard<std::mutex> lock(usb_queue_mutex_);
        usb_queue_.pop_front();
      }
    }
  }

  USB_DRIVER_LOCAL
  void handle_usb_events_callback()
  {
    if (!keep_running_) {
      RCLCPP_WARN(get_logger(), "shutting down handling of usb events ...");
      handle_usb_events_timer_->cancel();
      return;
    }

    RCLCPP_DEBUG_ONCE(get_logger(), "initial handle_usb_events_callback ...");
    if (usbc_ == nullptr) {
      return;
    }

    // Only attempt to process events if connected and not in error
    if (usbc_->driver_state() == usb::USBDriverState::DISCONNECTED) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 100,
        "handle_usb_events_callback - usb disconnected (waiting) ....");
      // return;
    }

    if (usbc_->driver_state() == usb::USBDriverState::ERROR) {
      RCLCPP_DEBUG(get_logger(), "handle_usb_events_callback - usb connection in error state!");
      return;
    }

    RCLCPP_DEBUG(get_logger(), "start handle_usb_events");
    try {
      usbc_->handle_usb_events();
    } catch (usb::UsbException & e) {
      RCLCPP_ERROR(this->get_logger(), "handle usb events UsbException: %s", e.what());
    } catch (std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "handle usb events exception: %s", e.what());
    } catch (const char * msg) {
      RCLCPP_ERROR(this->get_logger(), "handle usb events - %s", msg);
    } catch (const std::string & msg) {
      RCLCPP_ERROR(this->get_logger(), "handle usb events - %s", msg.c_str());
    }
    ;

    RCLCPP_DEBUG(get_logger(), "finish handle_usb_events");
  }
};
}  // namespace usb_driver

RCLCPP_COMPONENTS_REGISTER_NODE(usb_driver::UsbNode)
