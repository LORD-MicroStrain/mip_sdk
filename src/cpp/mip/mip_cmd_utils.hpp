#pragma once

#include "mip_interface.hpp"

#include "definitions/commands_base.hpp"
#include "definitions/commands_3dm.hpp"


namespace mip
{
    TypedResult<commands_base::SetIdle> setIdle(C::mip_interface& device);

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Utility function to apply a setting and immediately save it.
    ///
    /// This is functionally similar to runCommand, except:
    /// 1) The command must have a function selector,
    /// 2) The setting is immediately saved, and
    /// 3) The function selector is modified in this function so cmd isn't const.
    ///
    ///@tparam Cmd Command type (let the compiler deduce this).
    ///
    ///@param device
    ///@param cmd Setting command to send (similar to runCommand).
    ///
    ///@returns CmdResult
    ///
    template<class Cmd>
    TypedResult<Cmd> setAndSave(C::mip_interface& device, Cmd& cmd)
    {
        cmd.function = mip::FunctionSelector::WRITE;
        auto result = runCommand(device, cmd);
        if(!result)
            return result;
        cmd.function = mip::FunctionSelector::SAVE;
        return runCommand(device, cmd, 200);
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Utility function to load a setting and read it back.
    ///
    /// This is effectively the opposite of setAndSave, and works in basically the
    /// same way but in reverse.
    ///
    ///@tparam Cmd Command type (let the compiler deduce this).
    ///@tparam Rsp Response type (let the compiler deduce this).
    ///
    ///@param device
    ///@param cmd Setting command to send (similar to runCommand).
    ///@param[out] rsp Response to cmd (typically of type `typename Cmd::Response`)
    ///
    ///@returns CmdResult
    ///
    template<class Cmd, class Rsp=typename Cmd::Response>
    TypedResult<Cmd> loadAndRead(C::mip_interface& device, Cmd& cmd, Rsp& rsp)
    {
        cmd.function = mip::FunctionSelector::LOAD;
        auto result = runCommand(device, cmd);
        if(!result)
            return result;
        cmd.function = mip::FunctionSelector::READ;
        return runCommand(device, cmd, rsp);
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Same as loadAndRead except resets to the default instead of loading
    ///       the stored setting.
    ///
    ///@tparam Cmd Command type (let the compiler deduce this).
    ///@tparam Rsp Response type (let the compiler deduce this).
    ///
    ///@param device
    ///@param cmd Setting command to send (similar to runCommand).
    ///@param[out] rsp Response to cmd (typically of type `typename Cmd::Response`)
    ///
    ///@returns CmdResult
    ///
    template<class Cmd, class Rsp=typename Cmd::Response>
    TypedResult<Cmd> resetAndRead(C::mip_interface& device, Cmd& cmd, Rsp& rsp)
    {
        cmd.function = mip::FunctionSelector::RESET;
        auto result = runCommand(device, cmd);
        if(!result)
            return result;
        cmd.function = mip::FunctionSelector::READ;
        return runCommand(device, cmd, rsp);
    }

    ////////////////////////////////////////////////////////////////////////////////
    ///@brief Polls the device for a specific data quantity using the 3DM Poll cmd.
    ///
    ///@tparam DataField Data type to retrieve (let the compiler deduce this).
    ///
    ///@param device
    ///@param wait_time Approximate maximum time to wait for the data.
    ///@param[out] field Field data from the device, if result is ACK.
    ///
    ///@returns CmdResult::ACK_OK - If the data was received successfully.
    ///@returns CmdResult::STATUS_TIMEDOUT - If the data was not received within wait_time.
    ///@returns CmdResult::STATUS_ERROR - If an update() call failed.
    ///@returns Any other code returned from a command or update call.
    ///
    template<class DataField>
    TypedResult<mip::commands_3dm::PollData> pollData(Interface& device, Timeout wait_time, DataField& field)
    {
        struct Context
        {
            bool found = false;
            DataField& fieldRef;

            void callback(const mip::FieldView& raw, mip::Timestamp)
            {
                found = raw.extract(fieldRef);
            }
        };
        Context ctx{false, field};

        const Timestamp start_time = device.lastUpdateTime();

        DispatchHandler handler;
        device.registerFieldCallback<Context, &Context::callback>(
            handler, DataField::DESCRIPTOR_SET, DataField::FIELD_DESCRIPTOR, &ctx
        );

        const uint8_t descriptors = DataField::FIELD_DESCRIPTOR;

        auto result = commands_3dm::pollData(device, DataField::DESCRIPTOR_SET, false, 1, &descriptors);

        if(result)
        {
            while(!ctx.found)
            {
                const Timeout delta = device.lastUpdateTime() - start_time;

                if(delta >= wait_time)
                {
                    result = mip::CmdResult::STATUS_TIMEDOUT;
                    break;
                }

                if(!device.update(wait_time - delta, false))
                {
                    result = mip::CmdResult::STATUS_ERROR;
                    break;
                }
            }
        }

        device.removeDispatcher(handler);
        return result;
    }

/*
    template<class DataField0, class... DataFields>
    TypedResult<mip::commands_3dm::PollData> pollData(C::mip_interface& device, DataField0& field0, DataFields&... fields)
    {
        static_assert(((DataField0::DESCRIPTOR_SET == DataFields::DESCRIPTOR_SET) && ...), "All fields must have the same descriptor set");

        constexpr uint8_t descriptors[1+sizeof...(DataFields)] = {
            DataField0::FIELD_DESCRIPTOR,
            DataFields::FIELD_DESCRIPTOR...
        };

        auto callback = [](void* p)

        mip::DispatchHandler handler;
        static_cast<Interface&>(device).registerFieldCallback(handler, DataField0::DESCRIPTOR_SET, mip::C::MIP_DISPATCH_ANY_DESCRIPTOR, callback);

        mip::DispatchHandler handlers[1+sizeof...(DataFields)];
        (static_cast<Interface&>(device).registerFieldCallback(handlers, DataField0::DESCRIPTOR_SET, DataFields::FIELD_DESCRIPTOR), ...);

        auto result = mip::commands_3dm::pollData(device, DataField0::DESCRIPTOR_SET, false, sizeof(descriptors)/sizeof(descriptors[0]), descriptors);

        (static_cast<Interface&>(device))

        return result;
    }
*/
} // namespace mip
