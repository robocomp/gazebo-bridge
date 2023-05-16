//
// Created by Sergio Eslava Velasco on 2/05/23.
//

// Important to include the plugin's header.
#include "EntitiesControl.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace std;


EntitiesControl::EntitiesControl(){
}

EntitiesControl::~EntitiesControl() = default;


class EntitiesControlCommandBase {

};


#pragma region Gazebo Execution Flow

//////////////////////////////////////////////////
void EntitiesControl::Configure(const gz::sim::Entity &_entity,
                                const std::shared_ptr<const sdf::Element> &_sdf,
                                gz::sim::EntityComponentManager &_ecm,
                                gz::sim::EventManager &/*_eventMgr*/) {

    // We save the .sdf and the ECSystem in memory for future use.
    sdfConfig = _sdf->Clone();
    ecm = &_ecm;
    model = Model(_entity);

    // Status Message
    gzmsg << "[" << HEADER_NAME << "] Configured." << std::endl;

}

//////////////////////////////////////////////////////////////
void EntitiesControl::PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) {

    // Initialization settings
    if (!initialized) {
        // We call Load here instead of Configure because we can't be guaranteed
        // that all entities have been created when Configure is called
        Load(_ecm, sdfConfig);

        initialized = true;
    }

}

void EntitiesControl::Update(const gz::sim::UpdateInfo &_info,
            gz::sim::EntityComponentManager &_ecm){
    
}


// Here we implement the PostUpdate function, which is called at every iteration.
void EntitiesControl::PostUpdate(const gz::sim::UpdateInfo &_info,
                                 const gz::sim::EntityComponentManager &/*_ecm*/) {

}
//////////////////////////////////////////////////////////////
#pragma endregion Gazebo Execution Flow


void EntitiesControl::Load(const gz::sim::EntityComponentManager &_ecm,
                           const sdf::ElementPtr &_sdf) {

    ///////////////////////////////////////////////////////
    ////// CREATION OF GetWorldPosition TOPIC /////////////
    ///////////////////////////////////////////////////////

    std::vector<Entity> modelEntities = model.Models(_ecm);

    for (int i = 0; i < modelEntities.size(); ++i) {

        Model model = Model(modelEntities[i]);

        auto topic = ("/model/" + model.Name(_ecm) + "/get_world_position");

        gz::transport::Node::Publisher getWorldPositionPub = node.Advertise<gz::msgs::Pose>(topic);

        publishers.push_back(getWorldPositionPub);

        gzmsg << "[" << HEADER_NAME << "] Topic: " << topic << " created." << std::endl;
    }

}

void EntitiesControl::SetLinkLinearVelocity(EntityComponentManager &_ecm,
                                            sdf::ElementPtr _sdf,
                                            const std::string &_linkName,
                                            const gz::math::Vector3d &_linearVelocity) {
    Entity entity = _ecm.EntityByComponents(components::Name(_linkName));

    if (!entity)
        gzerr << "[" << HEADER_NAME << "] Link not found." << endl;

    Link link(entity);

    link.SetLinearVelocity(_ecm, _linearVelocity);
}

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
GZ_ADD_PLUGIN(
        EntitiesControl,
        System,
        EntitiesControl::ISystemConfigure,
        EntitiesControl::ISystemPostUpdate,
        EntitiesControl::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(EntitiesControl, "gz::sim::systems::EntitiesControl")