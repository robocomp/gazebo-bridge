//
// Created by usuario on 2/05/23.
//

#ifndef GAZEBO_BRIDGE_ENTITIESCONTROL_HH
#define GAZEBO_BRIDGE_ENTITIESCONTROL_HH

// The only required include in the header is this one.
// All others will depend on what your plugin does.
#include <gz/sim/System.hh>

// It's good practice to use a custom namespace for your project.
namespace entities_control {

    // Forward declarations
    class EntitiesControlPrivate;

    // This is the main plugin's class. It must inherit from System and at least
    // one other interface.
    // Here we use `ISystemPostUpdate`, which is used to get results after
    // physics runs. The opposite of that, `ISystemPreUpdate`, would be used by
    // plugins that want to send commands.
    // "ISystemConfigure" is used to set the initial values of our private class
    class EntitiesControl:
            public gz::sim::System,
            public gz::sim::ISystemPostUpdate,
            public gz::sim::ISystemConfigure,
            public gz::sim::ISystemPreUpdate
    {
        /// \brief Constructor
    public: EntitiesControl();

        // Plugins inheriting ISystemPostUpdate must implement the PostUpdate
        // callback. This is called at every simulation iteration after the physics
        // updates the world. The _info variable provides information such as time,
        // while the _ecm provides an interface to all entities and components in
        // simulation.
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                            const gz::sim::EntityComponentManager &_ecm) override;

    public: void Configure(const gz::sim::Entity &_id,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) final;

    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm) override;

        /// Private data pointer
    private: std::unique_ptr<EntitiesControlPrivate> dataPtr;
    };

}

#endif //GAZEBO_BRIDGE_ENTITIESCONTROL_HH
