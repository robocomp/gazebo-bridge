//
// Created by Sergio Eslava Velasco on 2/05/23.
//

// Important to include the plugin's header.
#include "EntitiesControl.hh"

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
GZ_ADD_PLUGIN(
        entities_control::EntitiesControl,
        gz::sim::System,
        entities_control::EntitiesControl::ISystemConfigure,
        entities_control::EntitiesControl::ISystemPostUpdate,
        entities_control::EntitiesControl::ISystemPreUpdate)

using namespace gz;
using namespace sim;
using namespace systems;
using namespace std;

using namespace entities_control;

EntitiesControl::EntitiesControl()
{
}


#pragma region Gazebo Execution Flow
//////////////////////////////////////////////////
void EntitiesControl::Configure(const gz::sim::Entity &_entity,
                                const std::shared_ptr<const sdf::Element> &_sdf,
                                gz::sim::EntityComponentManager &_ecm,
                                gz::sim::EventManager &/*_eventMgr*/)
{
    // We save the .sdf and the ECSystem in memory for future use.
    sdfConfig = _sdf->Clone();
    ecm = &_ecm;

    ///////////////////////////////////////////////////////
    ////// CREATION OF SetLinkLinearVelocity SERVICE //////
    ///////////////////////////////////////////////////////

    // Getting the name of the world
    const components::Name *constCmp = _ecm.Component<components::Name>(_entity);
    const std::string &worldName = constCmp->Data();

    auto validWorldName = transport::TopicUtils::AsValidTopic(worldName);
    if (validWorldName.empty())
    {
        gzerr << "World name [" << worldName
              << "] doesn't work well with transport, services not advertised."
              << std::endl;
        return;
    }

    // Create service
    std::string serviceName{"/world/" + validWorldName + "/set_link_linear_velocity"};
    node.Advertise(serviceName, &EntitiesControl::SetLinkLinearVelocityService, this);

    gzmsg << "[" << HEADER_NAME << "]" << " Service created: " << "[" << serviceName << "]" << std::endl;

    ///////////////////////////////////////////////////////

    // Status Message
    gzmsg << "[" << HEADER_NAME << "] Configured." << endl;
}

//////////////////////////////////////////////////
// Here we implement the PostUpdate function, which is called at every iteration.
void EntitiesControl::PostUpdate(const gz::sim::UpdateInfo &_info,
                                 const gz::sim::EntityComponentManager &/*_ecm*/)
{

}

void EntitiesControl::PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm)
{
    // Initialization settings
    if (!initialized)
    {
        // We call Load here instead of Configure because we can't be guaranteed
        // that all entities have been created when Configure is called
        Load(_ecm, sdfConfig);

        initialized = true;
    }

    //SetLinkLinearVelocity(const_cast<EntityComponentManager &>(_ecm), sdfConfig, "link_robocomp", gz::math::Vector3d(0.0, -1.0, 0.0));
}

#pragma endregion Gazebo Execution Flow


void EntitiesControl::Load(const gz::sim::EntityComponentManager &_ecm,
                             const sdf::ElementPtr &_sdf) {

}

void EntitiesControl::SetLinkLinearVelocity(EntityComponentManager& _ecm,
                                              sdf::ElementPtr _sdf,
                                              const std::string& _linkName,
                                              const gz::math::Vector3d& _linearVelocity)
{
    Entity entity = _ecm.EntityByComponents(components::Name(_linkName));

    if (!entity)
        gzerr << "[" << HEADER_NAME << "] Link not found." << endl;

    Link link(entity);

    link.SetLinearVelocity(_ecm, _linearVelocity);
}

#pragma region Public Interfaces

bool EntitiesControl::SetLinkLinearVelocityService(const gz::msgs::Pose &_req, gz::msgs::Boolean &_res){

    const std::string& linkName = _req.name();
    const auto& vel = _req.position();
    gz::math::Vector3<double> linearVelocity(vel.x(), vel.y(), vel.z());

    // TODO: No es buen aproach llamar directamente a la función sin pasar por el flujo de ejecución de Gazebo,
    // Probar con asignar atributos de clase y que las llamadas al método se hagan desde el PreUpdate
    // Quizás la solución es que entre la llamada en una lista y el PreUpdate ejecute la lista.
    // SetLinkLinearVelocity(*ecm, sdfConfig, linkName, linearVelocity);
}

#pragma endregion Public Interfaces

