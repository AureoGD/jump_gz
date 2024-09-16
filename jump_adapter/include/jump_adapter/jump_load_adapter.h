#ifndef JUMP_LOAD_ADAPTER_HH_
#define JUMP_LOAD_ADAPTER_HH_

#include <string>
#include <thread>

#include <gz/launch/Plugin.hh>
#include <gz/plugin/Register.hh>

#include "jump_adapter/jump_adapter.h"

namespace gz
{
    namespace launch
    {
        class JumpLoadAdapter : public gz::launch::Plugin
        {
        public:
            JumpLoadAdapter();

            virtual ~JumpLoadAdapter();

            virtual bool Load(
                const tinyxml2::XMLElement *_elem) override final;

        private:
            void Run();
            JumpAdapter jumpAdapter;
        };
    }
}
// Register the plugin
GZ_ADD_PLUGIN(gz::launch::JumpLoadAdapter, gz::launch::Plugin)

#endif