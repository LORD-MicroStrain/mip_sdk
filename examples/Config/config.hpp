#pragma once

#include "setting.hpp"

#include <mip/extras/composite_result.hpp>

#include <memory>


class Config
{
public:
    Config(mip::Interface& device) : mDevice(device) {}

    template<class MipCmd>
    void registerSetting() { mSettings.push_back(std::unique_ptr<Setting<MipCmd>>(new Setting<MipCmd>())); }

    template<class MipCmd, typename... Args>
    void registerSetting(uint16_t instance, Args... args) { mSettings.push_back(std::make_unique<MultiSetting<MipCmd>>(instance, args...)); }

    auto begin() const { return mSettings.begin(); }
    auto end() const { return mSettings.end(); }

    mip::CompositeResult applyAll();
    mip::CompositeResult applyChanged();
    mip::CompositeResult readAll();
    mip::CompositeResult saveAll();
    mip::CompositeResult loadAll();
    mip::CompositeResult resetAll();

private:
    mip::Interface& mDevice;
    std::vector<std::unique_ptr<SettingBase>> mSettings;
};
