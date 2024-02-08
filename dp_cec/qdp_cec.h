/*
* Copyright (c) 2014, 2020-2021, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of The Linux Foundation. nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
* ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Changes from Qualcomm Innovation Center are provided under the following license:
*
* Copyright (c) 2022, 2024 Qualcomm Innovation Center, Inc. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted (subject to the limitations in the
* disclaimer below) provided that the following conditions are met:
*
*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*
*    * Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.
*
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
* GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
* HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
* IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
* OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef QDP_CEC_H
#define QDP_CEC_H

#include <hardware/hdmi_cec.h>
#include <sys/poll.h>
#include <sys/prctl.h>
#include <sys/socket.h>
#include <sys/resource.h>
#include <linux/netlink.h>
#include <thread>
#include <vector>
#include <cutils/properties.h>

#define HDMI_CEC_HARDWARE_INTERFACE1 "hdmi_cec_hw_if1"
#define HDMI_CEC_HARDWARE_INTERFACE2 "hdmi_cec_hw_if2"
#define HDMI_CEC_HARDWARE_INTERFACE3 "hdmi_cec_hw_if3"
#define HDMI_CEC_HARDWARE_INTERFACE4 "hdmi_cec_hw_if4"
#define HDMI_CEC_HARDWARE_INTERFACE5 "hdmi_cec_hw_if5"

namespace qdpcec {

struct cec_callback_t {
    // Function in HDMI service to call back on CEC messages
    event_callback_t callback_func;
    // This stores the object to pass back to the framework
    void* callback_arg;

};

struct eventData;

struct cec_node_t {
    int fd = -1;
    int node_num = 0;
    const char *device = NULL;
    bool is_connected = false;
    unsigned caps = -1;
    unsigned available_log_addrs = -1;
    unsigned num_log_addrs = -1;
    __u16 log_addr_mask = -1;
    __u16 phys_addr = -1;
    __u8 log_addr[CEC_MAX_LOG_ADDRS] = {};
};

struct cec_context_t {
    hdmi_cec_device_t device;    // Device for HW module
    cec_node_t node;
    cec_callback_t callback;     // Struct storing callback object
    bool enabled = false;
    bool arc_enabled = false;
    bool system_control = false;         // If true, HAL/driver handle CEC messages
    hdmi_port_info *port_info;   // HDMI port info

    // Logical address is stored in an array, the index of the array is the
    // logical address and the value in the index shows whether it is set or not
    int logical_address[CEC_ADDR_BROADCAST] = {};
    int prim_log_addr = CEC_ADDR_UNREGISTERED;
    int version = -1;
    uint32_t vendor_id = -1;

    std::vector<pollfd> poll_fds = {};          // poll fds for cec message monitor and exit signal
                                                // on cec message monitor thread
    int exit_fd = -1;
    bool cec_exit_thread = false;
    std::thread cec_monitor;                    // cec monitor thread variable

    std::vector<std::string> node_list = {};
    std::vector<eventData> event_data_list = {};
};

struct eventData {
    const char* event_name = NULL;
    void (*event_parser)(cec_context_t* ctx, uint32_t node_event) = NULL;
};

void cec_receive_message(cec_context_t *ctx, char *msg, ssize_t len);
void cec_hdmi_hotplug(cec_context_t *ctx, int connected);
/**
 * cec_msg_initiator - return the initiator's logical address.
 * @msg:»       the message structure
 */
static inline __u8 cec_msg_initiator(const struct cec_msg *msg)
{
       return msg->msg[0] >> 4;
}

/**
 * cec_msg_destination - return the destination's logical address.
 * @msg:»       the message structure
 */
static inline __u8 cec_msg_destination(const struct cec_msg *msg)
{
       return msg->msg[0] & 0xf;
}

/**
 * cec_msg_opcode - return the opcode of the message, -1 for poll
 * @msg:»       the message structure
 */
static inline int cec_msg_opcode(const struct cec_msg *msg)
{
       return msg->len > 1 ? msg->msg[1] : -1;
}

/**
 * cec_msg_is_broadcast - return true if this is a broadcast message.
 * @msg:»       the message structure
 */
static inline int cec_msg_is_broadcast(const struct cec_msg *msg)
{
       return (msg->msg[0] & 0xf) == 0xf;
}

/**
 * cec_msg_init - initialize the message structure.
 * @msg:»       the message structure
 * @initiator:»       the logical address of the initiator
 * @destination:the logical address of the destination (0xf for broadcast)
 *
 * The whole structure is zeroed, the len field is set to 1 (i.e. a poll
 * message) and the initiator and destination are filled in.
 */
static inline void cec_msg_init(struct cec_msg *msg,
                            __u8 initiator, __u8 destination)
{
       memset(msg, 0, sizeof(*msg));
       msg->msg[0] = (initiator << 4) | destination;
       msg->len = 1;
}

/**
 * cec_msg_set_reply_to - fill in destination/initiator in a reply message.
 * @msg:»       the message structure for the reply
 * @orig:»       the original message structure
 *
 * Set the msg destination to the orig initiator and the msg initiator to the
 * orig destination. Note that msg and orig may be the same pointer, in which
 * case the change is done in place.
 */
static inline void cec_msg_set_reply_to(struct cec_msg *msg,
                                   struct cec_msg *orig)
{
       /* The destination becomes the initiator and vice versa */
       msg->msg[0] = (cec_msg_destination(orig) << 4) |
                    cec_msg_initiator(orig);
       msg->reply = msg->timeout = 0;
}
}; //namespace
#endif /* end of include guard: QDP_CEC_H */
