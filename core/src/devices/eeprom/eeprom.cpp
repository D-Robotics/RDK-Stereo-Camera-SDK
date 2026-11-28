/**
 * @file eeprom.cpp
 * @brief EEPROM model registry table and probe loop.
 * @brief EEPROM 型号注册表与探测循环。
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "devices/eeprom/eeprom.hpp"

#include <vector>

namespace dsc {
namespace eeprom {

namespace {

std::vector<const ModelDesc *> &registry()
{
    static std::vector<const ModelDesc *> r;
    return r;
}

} // namespace

bool register_model(const ModelDesc *desc)
{
    registry().push_back(desc);
    return true;
}

Eeprom::Eeprom(uint8_t bus, uint8_t addr)
    : bus_(bus, addr)
{
    if(!bus_)return;

    for(const ModelDesc *m : registry()){
        if(!m->probe(bus_))continue;
        desc_ = m;
        return;
    }
}

Status Eeprom::read(StereoImuModel *out){return desc_->read(bus_, out);}

const char *Eeprom::name() const{return desc_->name;}

const char *Eeprom::info() const{return desc_->info;}

} // namespace eeprom
} // namespace dsc
