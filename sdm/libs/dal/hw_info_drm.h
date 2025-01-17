/*
* Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
*
* Copyright (c) 2022, 2024 Qualcomm Innovation Center, Inc. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above
*       copyright notice, this list of conditions and the following
*       disclaimer in the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of The Linux Foundation nor the names of its
*       contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
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
*/

#ifndef __HW_INFO_DRM_H__
#define __HW_INFO_DRM_H__

#include <core/core_interface.h>
#include <core/sdm_types.h>
#include <drm_interface.h>
#include <private/hw_info_types.h>
#include <private/hw_info_interface.h>
#include <bitset>
#include <vector>
#include <map>
#include <string>

namespace sdm {

class HWInfoDRM : public HWInfoInterface {
 public:
  virtual DisplayError Init();
  virtual ~HWInfoDRM();
  virtual DisplayError GetHWResourceInfo(HWResourceInfo *hw_resource);
  virtual DisplayError GetFirstDisplayInterfaceType(HWDisplayInterfaceInfo *hw_disp_info);
  virtual DisplayError GetDisplaysStatus(bool skip_reload, HWDisplaysInfo *hw_displays_info);
  virtual DisplayError GetMaxDisplaysSupported(DisplayType type, int32_t *max_displays);
  virtual DisplayError GetRequiredDemuraFetchResourceCount(
                       std::map<uint32_t, uint8_t> *required_demura_fetch_cnt);
  virtual DisplayError GetDemuraPanelIds(std::vector<uint64_t> *panel_ids);
  virtual DisplayError GetPanelBootParamString(std::string *panel_boot_param_string);
  virtual uint32_t GetMaxMixerCount();

 private:
  void Deinit();
  DisplayError GetHWRotatorInfo(HWResourceInfo *hw_resource);
  void GetSystemInfo(HWResourceInfo *hw_resource);
  void GetHWPlanesInfo(HWResourceInfo *hw_resource);
  void GetWBInfo(HWResourceInfo *hw_resource);
  DisplayError GetDynamicBWLimits(HWResourceInfo *hw_resource);
  void GetSDMFormat(uint32_t drm_format, uint64_t drm_format_modifier,
                    std::vector<LayerBufferFormat> *sdm_formats);
  void GetSDMFormat(uint32_t v4l2_format, LayerBufferFormat *sdm_format);
  void GetRotatorFormatsForType(int fd, uint32_t type,
                                std::vector<LayerBufferFormat> *supported_formats);
  DisplayError GetRotatorSupportedFormats(uint32_t v4l2_index, HWResourceInfo *hw_resource);
  void PopulateSupportedFmts(HWSubBlockType sub_blk_type, const sde_drm::DRMPlaneTypeInfo &info,
                             HWResourceInfo *hw_resource);
  void PopulateSupportedInlineFmts(const sde_drm::DRMPlaneTypeInfo &info,
                                   HWResourceInfo *hw_resource);
  void PopulatePipeCaps(const sde_drm::DRMPlaneTypeInfo &info, HWResourceInfo *hw_resource);
  void PopulatePipeBWCaps(const sde_drm::DRMPlaneTypeInfo &info, HWResourceInfo *hw_resource);
  void MapPlaneToConnector(HWResourceInfo *hw_resource);
  void GetInitialDemuraInfo(HWResourceInfo *hw_resource);

  sde_drm::DRMManagerInterface *drm_mgr_intf_ = {};
  bool default_mode_ = false;

  static const int kMaxStringLength = 1024;
  static const int kKiloUnit = 1000;

  static HWResourceInfo *hw_resource_;
};

}  // namespace sdm

#endif  // __HW_INFO_DRM_H__
