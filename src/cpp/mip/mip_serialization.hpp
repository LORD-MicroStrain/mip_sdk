#pragma once

#include <microstrain/common/serialization.hpp>

#include <type_traits>
#include <stddef.h>


namespace mip
{




template<class StructType, size_t I, typename... Ts>
size_t serialize_parameters_r(const std::tuple<Ts...>& args)
{
    constexpr size_t counter_offset = (StructType::COUNTER_PARAMS >> (2*I)) & 0b11;
    constexpr bool is_array = counter_offset != 0;

    // Non-arrays get appended
    if constexpr(!is_array)
        return serialize_parameters_r<StructType, I+1, Ts...>(args);

    //const size_t counter_index =
}


template<class FieldType, typename... Ts>
size_t serialize_parameters(std::tuple<Ts...>& args)
{

}


template<class T>
struct Parameter
{
    T& value;
};

template<class T, size_t MAX_SIZE_T, class Counter>
struct ArrayParameter : Parameter<T>
{
    Counter& count;
    static constexpr size_t max_count = MAX_SIZE_T;
};
template<class T, size_t MAX_SIZE_T>
struct ArrayParameter<T, MAX_SIZE_T, void> : Parameter<T>
{
    static constexpr size_t max_count = MAX_SIZE_T;
};

template<class T, class E, E SELECTED_VALUE_T>
struct UnionConditionalParameter : Parameter<T>
{
    const U& selector;

    bool matches() const { return selector == SELECTED_VALUE_T; }
};



//template<class FieldType, class Action, uint8_t FunctionValue>
//void apply(FieldType& field, Action action)
//{
//    auto values = field.as_tuple();
//}

} // namespace mip
