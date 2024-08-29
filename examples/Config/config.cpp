
#include "config.hpp"



mip::CompositeResult Config::applyAll()
{
    mip::CompositeResult results;

    for(auto& pSetting : mSettings)
    {
        results += pSetting->apply(mDevice);
    }

    return results;
}

mip::CompositeResult Config::applyChanged()
{
    mip::CompositeResult results;

    for(auto& pSetting : mSettings)
    {
        if(!pSetting->applied)
            results += pSetting->apply(mDevice);
    }

    return results;
}

mip::CompositeResult Config::readAll()
{
    mip::CompositeResult results;

    for(auto& pSetting : mSettings)
    {
        results += pSetting->read(mDevice);
    }

    return results;
}

mip::CompositeResult Config::saveAll()
{
    mip::CompositeResult results;

    for(auto& pSetting : mSettings)
    {
        results += pSetting->save(mDevice);
    }

    return results;
}

mip::CompositeResult Config::loadAll()
{
    mip::CompositeResult results;

    for(auto& pSetting : mSettings)
    {
        results += pSetting->load(mDevice);
    }

    return results;
}

mip::CompositeResult Config::resetAll()
{
    mip::CompositeResult results;

    for(auto& pSetting : mSettings)
    {
        results += pSetting->reset(mDevice);
    }

    return results;
}


