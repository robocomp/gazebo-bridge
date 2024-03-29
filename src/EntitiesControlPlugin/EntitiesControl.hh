//
// Created by Sergio Eslava Velasco on 2/05/23.
//

#ifndef GAZEBO_BRIDGE_ENTITIESCONTROL_HH
#define GAZEBO_BRIDGE_ENTITIESCONTROL_HH

// The only required include in the header is this one.
// All others will depend on what your plugin does.
#include <gz/sim/System.hh>
#include <vector>
#include <string>

#include <gz/common/Console.hh>
// This header is required to register plugins.
#include <gz/plugin/Register.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Link.hh>
#include <sdf/sdf.hh>

#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/SdfEntityCreator.hh"

#include <gz/transport/Node.hh>
#include <gz/transport/TopicUtils.hh>


namespace gz {

    namespace sim {

        namespace systems {

            const std::string HEADER_NAME = "EntitiesControl";

            class EntitiesControlPrivate;

            // This is the main plugin's class. It must inherit from System and at least
            // one other interface.
            // Here we use `ISystemPostUpdate`, which is used to get results after
            // physics runs. The opposite of that, `ISystemPreUpdate`, would be used by
            // plugins that want to send commands.
            // "ISystemConfigure" is used to set the initial values of our private class
            class EntitiesControl :
                    public System,
                    public ISystemConfigure,
                    public ISystemPreUpdate{

            public:

                /// Constructor
                explicit EntitiesControl();

                // Destructor
                ~EntitiesControl() final;

                void Configure(const gz::sim::Entity &_id,
                               const std::shared_ptr<const sdf::Element> &_sdf,
                               gz::sim::EntityComponentManager &_ecm,
                               gz::sim::EventManager &_eventMgr) final;

                // Plugins inheriting ISystemPreUpdate must implement the PreUpdate
                // callback. This is called at every simulation iteration before the physics
                // updates the world.
                // [_info] This provides information such as time,
                // [_ecm] This provides an interface to all entities and components in simulation.
                void PreUpdate(const gz::sim::UpdateInfo &_info,
                               gz::sim::EntityComponentManager &_ecm) override;


            private:

                std::unique_ptr<EntitiesControlPrivate> dataPtr;
            };
        }
    }
}
#endif //GAZEBO_BRIDGE_ENTITIESCONTROL_HH
