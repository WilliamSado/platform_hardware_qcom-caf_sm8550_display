/*
* Copyright (c) 2014, 2016-2017, 2020-2021, The Linux Foundation. All rights reserved.
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
* Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
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

#define DEBUG 0
#define ATRACE_TAG (ATRACE_TAG_GRAPHICS | ATRACE_TAG_HAL)
#include <log/log.h>
#include <errno.h>
#include <hardware/hdmi_cec.h>
#include <utils/Trace.h>
#include <utils/debug.h>
#include <utils/sys.h>
#include <sys/ioctl.h>
#include <vector>
#include <linux/cec.h>
#include <linux/cec-funcs.h>
#include <cutils/properties.h>
#include "qdp_cec.h"

#define HWC_UEVENT_DRM_EXT_HOTPLUG "mdss_mdp/drm/card"
#define HDMI_HPD_STATE_PATH "/sys/bus/i2c/devices/i2c-0/0-002b/get_hpd_stat"

namespace qdpcec {

const int NUM_HDMI_PORTS = 1;
const int MAX_SEND_MESSAGE_RETRIES = 1;
const int MAX_CEC_DEVICES = 6;

const int MAX_PATH_LENGTH = 128;
const char* CEC_PATH_BASE = "/dev/cec";

enum {
    LOGICAL_ADDRESS_SET   =  1,
    LOGICAL_ADDRESS_UNSET = -1,
};

//Forward declarations
static int cec_init_device(cec_context_t *ctx, int hardware_intf);
static int cec_close_device(cec_context_t *ctx);
static void cec_close_context(cec_context_t* ctx __unused);
static int cec_enable(cec_context_t *ctx, int enable);
static int cec_is_connected(const struct hdmi_cec_device* dev, int port_id);
static void cec_monitor_deinit(cec_context_t* ctx);
static void handle_cec_msg_event(cec_context_t* ctx, uint32_t node_event);

void event_monitor(cec_context_t* ctx);  // hdmi event monitor function

static int populate_event_data(cec_context_t* ctx, std::vector<eventData> *event_data_list);
static int set_event_params(cec_context_t* ctx, uint32_t node_event, eventData *event_data);
static void handle_exit_event(cec_context_t* ctx, uint32_t node_event);
static void handle_hotplug_event(cec_context_t* ctx, uint32_t node_event);
static const char *get_event_value(const char *uevent_data, int length, const char *event_info);
static int uevent_init(int *uevent_fd);

/*
* Kernel can't report initial HPD state, because when HDMI
* driver is initialized HAL has not yet created cec driver's
* adapter devnode. HDMI HPD state can't be reported without
* adapter devnode, so we should get HDMI initial HPD state
* through HDMI HPD state node when cec adapter just finished
* initialization.
*/
static bool get_hpd_state_from_node()
{
   int ret = -1, fd = -1;
   char buf[2];
   ALOGI("%s", __func__);
   memset(buf, 0, sizeof(buf));
   fd = open(HDMI_HPD_STATE_PATH, O_RDONLY);
   if (fd < 0) {
       ALOGE("%s(): Open cec hpd state failed!", __func__);
       ALOGE("%s(): Error code: %d, %s\n", __func__, errno, strerror(errno));
       return false;
   }
   ret = read(fd, buf, sizeof(buf));
   close(fd);
   if (ret < 0) {
       ALOGE("Read hdmi hpd state failed\n");
       return false;
   }
   if(!strcmp(buf, "1")) {
       return true;
   } else if (!strcmp(buf, "0")) {
       return false;
   }
   ALOGE("%s can't get hdmi status HDMI_NOT_CONNECTED\n", __func__);
   return false;
}


static int cec_add_logical_address(const struct hdmi_cec_device* dev,
        cec_logical_address_t addr)
{
    int err = 0;

    if (addr <  CEC_ADDR_TV || addr > CEC_ADDR_BROADCAST) {
        ALOGE("%s: Received invalid address: %d ", __FUNCTION__, addr);
        return -EINVAL;
    }
    cec_context_t* ctx = (cec_context_t*)(dev);

    struct cec_log_addrs laddrs = {};
    __u8 all_dev_types = 0;
    __u8 prim_type = 0xff;
    __u8 la_type;
    ioctl(ctx->node.fd, CEC_ADAP_S_LOG_ADDRS, &laddrs);
    memset(&laddrs, 0, sizeof(laddrs));

    // TODO: Verify Version and Vendor ID
    laddrs.cec_version = CEC_OP_CEC_VERSION_1_4;
    laddrs.vendor_id = 0x000c03;

    switch(addr) {
        case CEC_ADDR_AUDIO_SYSTEM:
            prim_type = CEC_OP_PRIM_DEVTYPE_AUDIOSYSTEM;
            la_type = CEC_LOG_ADDR_TYPE_AUDIOSYSTEM;
            all_dev_types = CEC_OP_ALL_DEVTYPE_AUDIOSYSTEM;
            break;
        case CEC_ADDR_PLAYBACK_1:
        case CEC_ADDR_PLAYBACK_2:
        case CEC_ADDR_PLAYBACK_3:
            prim_type = CEC_OP_PRIM_DEVTYPE_PLAYBACK;
            la_type = CEC_LOG_ADDR_TYPE_PLAYBACK;
            all_dev_types = CEC_OP_ALL_DEVTYPE_PLAYBACK;
            break;
        case CEC_ADDR_RECORDER_1:
        case CEC_ADDR_RECORDER_2:
        case CEC_ADDR_RECORDER_3:
            prim_type = CEC_OP_PRIM_DEVTYPE_RECORD;
            la_type = CEC_LOG_ADDR_TYPE_RECORD;
            all_dev_types = CEC_OP_ALL_DEVTYPE_RECORD;
            break;
        case CEC_ADDR_TUNER_1:
        case CEC_ADDR_TUNER_2:
        case CEC_ADDR_TUNER_3:
        case CEC_ADDR_TUNER_4:
            prim_type = CEC_OP_PRIM_DEVTYPE_TUNER;
            la_type = CEC_LOG_ADDR_TYPE_TUNER;
            all_dev_types = CEC_OP_ALL_DEVTYPE_TUNER;
            break;
        case CEC_ADDR_TV:
            prim_type = CEC_OP_PRIM_DEVTYPE_TV;
            la_type = CEC_LOG_ADDR_TYPE_TV;
            all_dev_types = CEC_OP_ALL_DEVTYPE_TV;
            break;
        default:
            prim_type = CEC_OP_PRIM_DEVTYPE_SWITCH;
            la_type = CEC_LOG_ADDR_TYPE_UNREGISTERED;
            all_dev_types = CEC_OP_ALL_DEVTYPE_SWITCH;
            break;

    }

    laddrs.log_addr_type[laddrs.num_log_addrs++] = la_type;

    for (unsigned i = 0; i < laddrs.num_log_addrs; i++) {
        laddrs.primary_device_type[i] = prim_type;
        laddrs.all_device_types[i] = all_dev_types;
    }

    if (ioctl(ctx->node.fd, CEC_ADAP_S_LOG_ADDRS, &laddrs)) {
        err = errno;
        ALOGE("%s: logical address allocation failed: error=%d ", __FUNCTION__, err);
        return -err;
    }

    ctx->logical_address[addr] = LOGICAL_ADDRESS_SET;
    if (ctx->prim_log_addr == CEC_ADDR_UNREGISTERED)
        ctx->prim_log_addr = addr;

    //XXX: We can get multiple logical addresses here but we can only send one
    // to the driver. Store locally for now
    ALOGI("%s: Allocated logical address: %d ", __FUNCTION__, addr);
    return (int) err;
}

static void cec_clear_logical_address(const struct hdmi_cec_device* dev)
{
    cec_context_t* ctx = (cec_context_t*)(dev);

    if (ctx->node.is_connected & !(ctx->node.caps & CEC_CAP_LOG_ADDRS)) {
        ALOGE("%s: Missing CEC_CAP_LOG_ADDRS capability", __FUNCTION__);
        return;
    }

    struct cec_log_addrs laddrs = { };
    if (ioctl(ctx->node.fd, CEC_ADAP_S_LOG_ADDRS, &laddrs)) {
        ALOGE("%s: ioctl CEC_ADAP_S_LOG_ADDRS failed: errno=%d ", __FUNCTION__, errno);
        return;
    }

    memset(ctx->logical_address, LOGICAL_ADDRESS_UNSET,
            sizeof(ctx->logical_address));
    ctx->node.num_log_addrs = 0;

    ALOGD_IF(DEBUG, "%s: Cleared logical addresses", __FUNCTION__);
}

static int cec_get_physical_address(const struct hdmi_cec_device* dev,
        uint16_t* addr)
{
    int err;
    cec_context_t* ctx = (cec_context_t*)(dev);

    if (!ctx->node.is_connected) {
        *addr = 0;
        ALOGD_IF(DEBUG, "%s: Physical Address: 0x%x", __FUNCTION__, *addr);
        return 0;
    }

    if (ioctl(ctx->node.fd, CEC_ADAP_G_PHYS_ADDR, addr)) {
        err = errno;
        ALOGE("%s: ioctl CEC_ADAP_G_PHYS_ADDR failed, errno=%d", __FUNCTION__, err);
        return -err;
    }

    ALOGD_IF(DEBUG, "%s: Physical Address: 0x%x", __FUNCTION__, *addr);
    return 0;
}

static int cec_send_message(const struct hdmi_cec_device* dev,
        const cec_message_t* msg)
{
    ATRACE_CALL();

    // TODO: Need to find replacement for the following check
    // if(cec_is_connected(dev, 0) <= 0)
    //     return HDMI_RESULT_FAIL;

    cec_context_t* ctx = (cec_context_t*)(dev);

    if (!(ctx->node.caps & CEC_CAP_TRANSMIT)) {
        ALOGE("%s: Missing CEC_CAP_TRANSMIT capability.", __FUNCTION__);
        return -ENOTTY;
    }

    ALOGD_IF(DEBUG, "%s: initiator: %d destination: %d length: %u",
            __FUNCTION__, msg->initiator, msg->destination,
            (uint32_t) msg->length);

    struct cec_msg cmsg;

    cec_msg_init(&cmsg, msg->initiator, msg->destination);

    if (msg->length > 0) {
        // Setting OPCODE
        cmsg.msg[cmsg.len++] = msg->body[0];

        memcpy(&cmsg.msg[cmsg.len], &msg->body[1],
                sizeof(char)*(msg->length - 1));
        cmsg.len += (msg->length - 1);
    }

    int retry_count = 0;
    int err = 0;
    // HAL spec requires us to retry at least once.
    while (true) {
        if (msg->initiator == msg->destination) {
            err = -ENXIO;
            ALOGD("%s: initiator and destination cannot be same.", __FUNCTION__);
            break;
        } else {
            if (ioctl(ctx->node.fd, CEC_TRANSMIT, &cmsg)) {
                err = -errno;
                ALOGE("%s: ioctl CEC_TRANSMIT failed. errno: %d", __FUNCTION__, errno);
            }
        }
        retry_count++;
        if (err == -EBUSY && retry_count <= MAX_SEND_MESSAGE_RETRIES) {
            ALOGE("%s: CEC line busy, retrying", __FUNCTION__);
        } else {
            break;
        }
    }

    int opcode_ret = cec_msg_opcode(&cmsg);

    ALOGI("%s: ioctl CEC_TRANSMIT - tx_status=%02x len=%d addr=%02x opcode=%02x",
            __FUNCTION__, cmsg.tx_status, cmsg.len, cmsg.msg[0], opcode_ret);

    if (err < 0) {
       if (err == -ENXIO) {
           ALOGI("%s: No device exists with the destination address",
                   __FUNCTION__);
           return HDMI_RESULT_NACK;
       } else if (err == -EBUSY) {
            ALOGE("%s: CEC line is busy, max retry count exceeded",
                    __FUNCTION__);
            return HDMI_RESULT_BUSY;
        } else {
            ALOGE("%s: Failed to send CEC message err: %d - %s",
                    __FUNCTION__, err, strerror(-err));
            return HDMI_RESULT_FAIL;
        }
    } else if (cmsg.tx_status & CEC_TX_STATUS_OK) {
        ALOGD_IF(DEBUG, "%s: Sent CEC message - %d bytes written",
                __FUNCTION__, err);
        return HDMI_RESULT_SUCCESS;
    }
    return HDMI_RESULT_NACK;
}

void cec_receive_message(cec_context_t *ctx, struct cec_msg *msg, ssize_t len)
{
    if(!ctx->system_control)
        return;

    hdmi_event_t event;
    event.type = HDMI_EVENT_CEC_MESSAGE;
    event.dev = (hdmi_cec_device *) ctx;
    event.cec.length = len - 1;
    event.cec.initiator = (cec_logical_address_t) cec_msg_initiator(msg);
    event.cec.destination = (cec_logical_address_t) cec_msg_destination(msg);
    // Copy opcode and operand
    size_t copy_size = event.cec.length > sizeof(event.cec.body) ?
                       sizeof(event.cec.body) : event.cec.length;
    memcpy(event.cec.body, &msg->msg[1], copy_size);

    ALOGI("%s: triggerring callback with received message", __FUNCTION__);
    ctx->callback.callback_func(&event, ctx->callback.callback_arg);
}

void cec_hdmi_hotplug(cec_context_t *ctx, int connected)
{
    //Ignore unplug events when system control is disabled
    if(!ctx->system_control && connected == 0)
        return;

    if (ctx->node.is_connected == connected) {
        ALOGW("Same hotplug event, ignoring.");
        return;
    }

    // initialise the device on connect
    ctx->node.is_connected = connected;
    if (connected) {
        cec_init_device(ctx, ctx->node.node_num);
    } else {
        cec_close_device(ctx);
    }

    // call callback
    hdmi_event_t event;
    event.type = HDMI_EVENT_HOT_PLUG;
    event.dev = (hdmi_cec_device *) ctx;
    event.hotplug.port_id = 1;
    event.hotplug.connected = connected ? HDMI_CONNECTED : HDMI_NOT_CONNECTED;
    if (ctx->callback.callback_func) {
        ctx->callback.callback_func(&event, ctx->callback.callback_arg);
    } else {
        ALOGW("HPD callback not registered!");
    }
}

static void cec_register_event_callback(const struct hdmi_cec_device* dev,
            event_callback_t callback, void* arg)
{
    ALOGD_IF(DEBUG, "%s: Registering callback", __FUNCTION__);
    cec_context_t* ctx = (cec_context_t*)(dev);
    ctx->callback.callback_func = callback;
    ctx->callback.callback_arg = arg;
}

static void cec_get_version(const struct hdmi_cec_device* dev, int* version)
{
    cec_context_t* ctx = (cec_context_t*)(dev);
    *version = ctx->version;
    ALOGD_IF(DEBUG, "%s: version: %d", __FUNCTION__, *version);
}

static void cec_get_vendor_id(const struct hdmi_cec_device* dev,
        uint32_t* vendor_id)
{
    cec_context_t* ctx = (cec_context_t*)(dev);
    *vendor_id = ctx->vendor_id;
    ALOGD_IF(DEBUG, "%s: vendor id: %u", __FUNCTION__, *vendor_id);
}

static void cec_get_port_info(const struct hdmi_cec_device* dev,
            struct hdmi_port_info* list[], int* total)
{
    ALOGD_IF(DEBUG, "%s: Get port info", __FUNCTION__);
    cec_context_t* ctx = (cec_context_t*)(dev);
    *total = NUM_HDMI_PORTS;
    *list = ctx->port_info;
}

static void cec_set_option(const struct hdmi_cec_device* dev, int flag,
        int value)
{
    cec_context_t* ctx = (cec_context_t*)(dev);
    switch (flag) {
        case HDMI_OPTION_WAKEUP:
            ALOGD_IF(DEBUG, "%s: Wakeup: value: %d", __FUNCTION__, value);
            //XXX
            break;
        case HDMI_OPTION_ENABLE_CEC:
            ALOGD_IF(DEBUG, "%s: Enable CEC: value: %d", __FUNCTION__, value);
            cec_enable(ctx, value? 1 : 0);
            break;
        case HDMI_OPTION_SYSTEM_CEC_CONTROL:
            ALOGD_IF(DEBUG, "%s: system_control: value: %d",
                    __FUNCTION__, value);
            ctx->system_control = !!value;
            break;
    }
}

static void cec_set_audio_return_channel(const struct hdmi_cec_device* dev,
        int port, int flag)
{
    cec_context_t* ctx = (cec_context_t*)(dev);
    ctx->arc_enabled = flag ? true : false;
    ALOGD_IF(DEBUG, "%s: ARC flag: %d port: %d", __FUNCTION__, flag, port);
}

static int cec_is_connected(const struct hdmi_cec_device* dev, int port_id)
{
    cec_context_t* ctx = (cec_context_t*)(dev);

    ALOGE("%s: is_connected=%d, port_id=%d", __FUNCTION__, ctx->node.is_connected, port_id);
    return ctx->node.is_connected;
}

static int cec_device_close(struct hw_device_t *dev)
{
    ALOGD_IF(DEBUG, "%s: Close CEC HAL ", __FUNCTION__);
    if (!dev) {
        ALOGE("%s: NULL device pointer", __FUNCTION__);
        return -EINVAL;
    }
    cec_context_t* ctx = (cec_context_t*)(dev);
    cec_close_context(ctx);
    free(dev);
    return 0;
}

// TODO: Fix implementation
static int cec_enable(cec_context_t *ctx, int enable)
{
    ctx->enabled = enable;
    return 0;
}

static int cec_close_device(cec_context_t *ctx)
{
    int ret = 0;

    if (ctx->node.fd > 0) {
        ALOGD_IF(DEBUG, "%s: closing cec device", __FUNCTION__);
        close(ctx->node.fd);
        ctx->node.fd = -1;

        // invalidate msg event poll
        eventData event_data;
	event_data.event_name = "cec_msg_event";
        ret = set_event_params(ctx, 0, &event_data);
    }
    return ret;
}

static int cec_init_device(cec_context_t *ctx, int hardware_intf)
{
    int err = -EINVAL;
    char value[PROPERTY_VALUE_MAX] = {0};
    int num = hardware_intf;

    char cec_dev_path[MAX_PATH_LENGTH];

    if(property_get("vendor.display.enable_cec_node", value, "-1") > 0) {
        num = atoi(value);
        if(num < 0 || num >= MAX_CEC_DEVICES)
            num = hardware_intf;
        else if(num != hardware_intf)
            return err;
        else
            ALOGI("%s: vendor.display.enable_cec_node property enabled!", __FUNCTION__);
    }

    snprintf(cec_dev_path, sizeof(cec_dev_path), "%s%d",
             CEC_PATH_BASE, num);
    ALOGD_IF(DEBUG, "%s: Trying num: %d cec_dev_path: %s", __FUNCTION__, num, cec_dev_path);

    if ((ctx->node.fd = open(cec_dev_path, O_RDWR)) >= 0) {
            ALOGD_IF(DEBUG, "%s: Found CEC device path at %s", __FUNCTION__, cec_dev_path);
            ctx->node.device = cec_dev_path;
    }

    if (ctx->node.fd < 0) {
        err = errno;
        ALOGE("%s: Failed to open CEC device (%s): error=%s",
                __FUNCTION__, cec_dev_path, strerror(err));
        return -err;
    }

    struct cec_caps caps = { };

    if (ioctl(ctx->node.fd, CEC_ADAP_G_CAPS, &caps)) {
        err = errno;
        ALOGE("%s: ioctl CEC_ADAP_G_CAPS failed: error=%s",
                __FUNCTION__, strerror(err));
        return -err;
    }

    ctx->node.caps = caps.capabilities;
    ctx->node.available_log_addrs = caps.available_log_addrs;

    // Set CEC Mode to wait for message.
    __u32 monitor = CEC_MODE_INITIATOR | CEC_MODE_FOLLOWER;

    if (ioctl(ctx->node.fd, CEC_S_MODE, &monitor)) {
        err = errno;
        ALOGE("%s: Selecting follower mode failed.\n: error=%s",
                __FUNCTION__, strerror(err));
        return -err;
    }

    // TODO: Enable CEC - framework expects it to be enabled by default
    cec_enable(ctx, true);

    eventData event_data;
    event_data.event_name = "cec_msg_event";
    err = set_event_params(ctx, 0, &event_data);

    ALOGD("%s: CEC enabled", __FUNCTION__);
    return 0;
}

//TODO: Create a cleanup function

static int cec_init_context(cec_context_t *ctx, int hardware_intf)
{
    int err = -EINVAL;

    ALOGD_IF(DEBUG, "%s: Initializing context", __FUNCTION__);

    ctx->node.fd = -1;

    // start monitor thread for connect event.
    ctx->node_list.push_back("cec_msg_event");
    ctx->node_list.push_back("hotplug_event");
    ctx->node_list.push_back("exit_event");

    err = populate_event_data(ctx, &ctx->event_data_list);
    if (err < 0) {
        ALOGE("Failed to populate poll parameters for monitoring HDMI CEC events. Exiting.");
        return err;
    }

    // TODO: check for failed to open error
    if (cec_init_device(ctx, hardware_intf) != 0) {
        ALOGE("Failed to open cec device, will wait for hpd.");
    }

    ctx->cec_monitor = std::thread(event_monitor, ctx);

    //Initialize ports - We support only one output port
    ctx->port_info = new hdmi_port_info[NUM_HDMI_PORTS];
    ctx->port_info[0].type = HDMI_OUTPUT;
    ctx->port_info[0].port_id = 1;
    ctx->port_info[0].cec_supported = 1;
    //XXX: Enable ARC if supported
    ctx->port_info[0].arc_supported = 0;
    ctx->node.node_num = hardware_intf;
    cec_get_physical_address((hdmi_cec_device *) ctx,
            &ctx->port_info[0].physical_address);

    ctx->version = 0x6;
    ctx->vendor_id = 0xA47733;
    cec_clear_logical_address((hdmi_cec_device_t*)ctx);
    ctx->node.is_connected = get_hpd_state_from_node();

    return 0;
}

static void cec_close_context(cec_context_t* ctx __unused)
{
    ALOGD("%s: Closing context", __FUNCTION__);

    uint64_t exit_value = 1;
    long int write_size = write(ctx->exit_fd, &exit_value, sizeof(uint64_t));

    if (write_size != sizeof(uint64_t)) {
        ALOGE("Error triggering exit_fd (%d). write size = %ld, error = %s",
            ctx->exit_fd, write_size, strerror(errno));
        return;
    }

    if (ctx->cec_monitor.joinable()) {
        ctx->cec_monitor.join();
    }
}

static int cec_device_open(const struct hw_module_t* module,
        const char* name,
        struct hw_device_t** device)
{
    ALOGD_IF(DEBUG, "%s: name: %s", __FUNCTION__, name);
    int status = -EINVAL;
    char buf[4];

    if (strlen(name) < strlen(HDMI_CEC_HARDWARE_INTERFACE)) {
        return status;
    }

    if (strncmp(name, HDMI_CEC_HARDWARE_INTERFACE, strlen(HDMI_CEC_HARDWARE_INTERFACE)))
        return status;

    snprintf(buf, sizeof(buf), "%s",
             name + strlen(HDMI_CEC_HARDWARE_INTERFACE));

    for (int i = 0; i < strlen(buf); i++)
        if (!isdigit(buf[i]))
            return status;

    int hardware_intf = atoi(buf);

    if (hardware_intf < MAX_CEC_DEVICES) {
        struct cec_context_t *dev;
        dev = (cec_context_t *) calloc (1, sizeof(*dev));
        if (dev) {
            status = cec_init_context(dev, hardware_intf);

            if (status < 0) {
                ALOGE("%s: Initializing failed.", __FUNCTION__);
                free(dev);
                return status;
            }

            //Setup CEC methods
            dev->device.common.tag       = HARDWARE_DEVICE_TAG;
            dev->device.common.version   = HDMI_CEC_DEVICE_API_VERSION_1_0;
            dev->device.common.module    = const_cast<hw_module_t* >(module);
            dev->device.common.close     = cec_device_close;
            dev->device.add_logical_address = cec_add_logical_address;
            dev->device.clear_logical_address = cec_clear_logical_address;
            dev->device.get_physical_address = cec_get_physical_address;
            dev->device.send_message = cec_send_message;
            dev->device.register_event_callback = cec_register_event_callback;
            dev->device.get_version = cec_get_version;
            dev->device.get_vendor_id = cec_get_vendor_id;
            dev->device.get_port_info = cec_get_port_info;
            dev->device.set_option = cec_set_option;
            dev->device.set_audio_return_channel = cec_set_audio_return_channel;
            dev->device.is_connected = cec_is_connected;

            *device = &dev->device.common;
            status = 0;
        } else {
            status = -EINVAL;
        }
    }
    return status;
}

void event_monitor(cec_context_t* ctx) {
    ALOGD("%s IN", __FUNCTION__);
    int err = -EINVAL;

    prctl(PR_SET_NAME, "cec_monitor", 0, 0, 0);
    setpriority(PRIO_PROCESS, 0, HAL_PRIORITY_URGENT_DISPLAY);

    while (!ctx->cec_exit_thread) {
        err = poll(ctx->poll_fds.data(), (nfds_t)ctx->event_data_list.size(), -1);
        if ( err <= 0 ) {
            ALOGI("Failed to poll, Error %s", strerror(errno));
            continue;
         }

         for (uint32_t event = 0; event < ctx->event_data_list.size(); event++) {
            pollfd &poll_fd = ctx->poll_fds[event];

            if (poll_fd.revents & POLLIN || poll_fd.revents & POLLPRI) {
                ctx->event_data_list[event].event_parser(ctx, event);
            }
        }
    }

    cec_monitor_deinit(ctx);
    ALOGD("%s OUT", __FUNCTION__);
    return;
}

static int populate_event_data(cec_context_t* ctx, std::vector<eventData> *event_data_list) {
    int err = -EINVAL;
    ctx->poll_fds.resize(ctx->node_list.size());

    for (uint32_t event = 0; event < ctx->node_list.size(); event++) {
        const char *event_name = ctx->node_list.at(event).c_str();
        eventData event_data;
        event_data.event_name = event_name;
        err = set_event_params(ctx, event, &event_data);
        if (err < 0) {
            ALOGE("Failed to set poll event parameters");
            return err;
        }

        event_data_list->push_back(event_data);
    }

    return 0;
}

static int set_event_params(cec_context_t* ctx, uint32_t node_event, eventData *event_data) {
    pollfd poll_fd = {-EINVAL, 0, 0};

    if (!strncmp(event_data->event_name, "cec_msg_event", strlen("cec_msg_event"))) {
        poll_fd.fd = ctx->node.fd;

        // FIXME: fd is not checked, this was by design
        poll_fd.events |= POLLIN | POLLPRI | POLLERR;
        event_data->event_parser = &handle_cec_msg_event;
    } else if (!strncmp(event_data->event_name, "exit_event", strlen("exit_event"))) {
        poll_fd.fd = eventfd(0, 0);
        poll_fd.events |= POLLIN;
        event_data->event_parser = &handle_exit_event;
        ctx->exit_fd = poll_fd.fd;
    } else if (!strncmp(event_data->event_name, "hotplug_event", strlen("hotplug_event"))) {
        if (!uevent_init(&poll_fd.fd)) {
            ALOGE("Failed to register uevent for hotplug detection");
            return -1;
        }

        poll_fd.events |= POLLIN | POLLERR;
        event_data->event_parser = &handle_hotplug_event;
    }

    ctx->poll_fds[node_event] = poll_fd;
    return 0;
}

static void handle_cec_msg_event(cec_context_t* ctx, uint32_t node_event) {
    if (ctx->poll_fds[node_event].revents & POLLIN) {
        ALOGD_IF(DEBUG, "Handling CEC message %s", __FUNCTION__);
        struct cec_msg msg = {};
        if (ioctl(ctx->node.fd, CEC_RECEIVE, &msg)) {
            ALOGE("%s: ioctl CEC_RECEIVE failed, err=%s", __FUNCTION__, strerror(errno));
        } else {
            ALOGI("%s: ioctl CEC_RECEIVE starts", __FUNCTION__);
            cec_receive_message(ctx, &msg, msg.len);
        }
    }

    if (ctx->poll_fds[node_event].revents & POLLPRI) {
        ALOGD_IF(DEBUG, "Handling DQEVENT message %s", __FUNCTION__);
        struct cec_event ev = {};
        if (ioctl(ctx->node.fd, CEC_DQEVENT, &ev)) {
            ALOGE("%s: ioctl CEC_DQEVENT failed, err=%s", __FUNCTION__, strerror(errno));
        } else {
            ALOGI("%s: ioctl CEC_DQEVENT event = %d", __FUNCTION__, ev.event);
        }
    }
}

static void handle_exit_event(cec_context_t* ctx, uint32_t node_event) {
    ALOGD_IF(DEBUG, "Enter %s", __FUNCTION__);

    if (ctx->poll_fds[node_event].revents & POLLIN) {
       ctx->cec_exit_thread = true;
    }

    return;
}

static void handle_hotplug_event(cec_context_t* ctx, uint32_t node_event) {
    char uevent_data[PAGE_SIZE];
    int count = 0;

    if (ctx->poll_fds[node_event].revents & POLLIN) {
        count = static_cast<int> (recv(ctx->poll_fds[node_event].fd, uevent_data,
            (INT32(sizeof(uevent_data))) - 2, 0));

        if ((count > 0) && (strcasestr(uevent_data, HWC_UEVENT_DRM_EXT_HOTPLUG))) {
            // MST hotplug will not carry connection status/test pattern etc.
            // Pluggable display handler will check all connection status
            // and take action accordingly.
            ALOGD("drm hotplug");
            const char *str_status = get_event_value(uevent_data, count, "status=");
            const char *str_mst = get_event_value(uevent_data, count, "MST_HOTPLUG=");
            if (!str_status && !str_mst) {
              return;
            }
            if (str_status) {
              bool connected = (strncmp(str_status, "connected", strlen("connected")) == 0);
              ALOGD("CEC is %s", connected ? "connected" : "disconnected");
              cec_hdmi_hotplug(ctx, connected);
            }
        }
    }

    return;
}

static const char *get_event_value(const char *uevent_data, int length, const char *event_info) {
    const char *iterator_str = uevent_data;
    const char *pstr = NULL;
    while (((iterator_str - uevent_data) <= length) && (*iterator_str)) {
        pstr = strstr(iterator_str, event_info);
        if (pstr) {
            break;
        }
        iterator_str += strlen(iterator_str) + 1;
    }

    if (pstr) {
           pstr = pstr + strlen(event_info);
    }

    return pstr;
}

/* Returns 0 on failure, 1 on success */
static int uevent_init(int *uevent_fd) {
    struct sockaddr_nl addr;
    int sz = 64*1024;
    int s;

    memset(&addr, 0, sizeof(addr));
    addr.nl_family = AF_NETLINK;
    addr.nl_pid = 0;
    addr.nl_groups = 0xffffffff;

    s = socket(PF_NETLINK, SOCK_DGRAM, NETLINK_KOBJECT_UEVENT);
    if (s < 0)
        return 0;

    setsockopt(s, SOL_SOCKET, SO_RCVBUFFORCE, &sz, sizeof(sz));

    if (bind(s, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
        close(s);
        return 0;
    }

    *uevent_fd = s;
    return (*uevent_fd > 0);
}

static void cec_monitor_deinit(cec_context_t* ctx) {
    for (uint32_t event = 0; event < ctx->poll_fds.size(); event++) {
        close(ctx->poll_fds[event].fd);
        ctx->poll_fds[event].fd = -1;
    }
}

}; //namespace qdpcec

// Standard HAL module, should be outside qdpcec namespace
static struct hw_module_methods_t cec_module_methods = {
        .open = qdpcec::cec_device_open
};

/* hw_cec module */
hw_module_t HAL_MODULE_INFO_SYM = {
    .tag = HARDWARE_MODULE_TAG,
    .version_major = 1,
    .version_minor = 0,
    .id = HDMI_CEC_HARDWARE_MODULE_ID,
    .name = "QTI HDMI CEC module",
    .author = "The Linux Foundation",
    .methods = &cec_module_methods,
};
