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

    /*
    optional<string> name = link.Name(_ecm);
    cout << "NOMBRE: " << name.value() << endl;
    */


    /*
 // Obtener un puntero al elemento "link" con el nombre especificado
 // TODO: Esto no funciona, no está devolviendo todos los links de la escena del pendulo.
 // Quizas sea porque el pendulo está en el .sdf a través un include????
 sdf::ElementPtr linkElem = _sdf->GetElement("link");


 cout << "" << linkElem->GetName() << endl;
 while (linkElem != nullptr) {

     if (linkElem->HasAttribute("name") && linkElem->GetAttribute("name")->GetAsString() == _linkName) {
         break;
     }

     // DEBUG //
     cout << linkElem->GetAttribute("name")->GetAsString() << endl;
     linkElem = linkElem->GetNextElement("link");
 }
 if (linkElem == nullptr) {
     gzerr << "No se encontró el elemento <link> con el nombre '" << _linkName << "'" << std::endl;
     return;
 }

 // Obtener el nombre del modelo y el link
 std::string modelName = linkElem->GetParent()->Get<std::string>("name");
 std::string linkName = linkElem->Get<std::string>("name");

 // Obtener la entidad correspondiente en el mundo de simulación
 Entity entity = _ecm.EntityByComponents(
         components::Name(linkName));

 if(entity == kNullEntity){
     gzerr << "No encontrada entidad en la simulación";
     return;
 }

 Link link(entity);

 // Establecer la velocidad lineal del enlace
 link.SetLinearVelocity(_ecm, _linearVelocity);
  */
}

#pragma region Public Interfaces

void EntitiesControl::SetLinkLinearVelocity(const std::string& _linkName, const gz::math::Vector3d& _linearVelocity){
    SetLinkLinearVelocity(*ecm, sdfConfig, _linkName, _linearVelocity);
}

#pragma endregion Public Interfaces

