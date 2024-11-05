#pragma once

#include <string>

namespace mip
{
    class FieldView;

    namespace metadata
    {
        class Definitions;
    }
}


extern mip::metadata::Definitions mipdefs;

std::string formatField(const mip::FieldView& field);
