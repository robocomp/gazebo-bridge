//
// Created by Sergio Eslava Velasco on 2/05/23.
//

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
#include <gz/transport/Node.hh>
#include <gz/transport/TopicUtils.hh>

// Important to include the plugin's header.
#include "EntitiesControl.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace std;

using namespace entities_control;

class entities_control::EntitiesControlPrivate{

    /// Initialize the plugin.
    /// [_ecm] Immutable reference to the EntityComponentManager.
    /// [_sdf] The SDF Element associated with this system plugin.
public: void Load(const EntityComponentManager &_ecm,
                  const sdf::ElementPtr &_sdf);

public: void SetLinkLinearVelocity(gz::sim::EntityComponentManager& _ecm,
                                   sdf::ElementPtr _sdf,
                                   const std::string& _linkName,
                                   const gz::math::Vector3d& _linearVelocity);

    /// Model interface
public: Model model{kNullEntity};

    /// The link entity
public: gz::sim::Link link;

    /// Copy of the sdf configuration used for this plugin
public: sdf::ElementPtr sdfConfig;

    /// Initialization flag.
public: bool initialized{false};

};

EntitiesControl::EntitiesControl()
        : dataPtr(std::make_unique<EntitiesControlPrivate>())
{
}

void EntitiesControlPrivate::Load(const EntityComponentManager &_ecm,
                             const sdf::ElementPtr &_sdf) {
    SetLinkLinearVelocity(const_cast<EntityComponentManager &>(_ecm), _sdf, "core", gz::math::Vector3d(0.0, -1.0, 0.0));
}

void EntitiesControlPrivate::SetLinkLinearVelocity(EntityComponentManager& _ecm,
                                              sdf::ElementPtr _sdf,
                                              const std::string& _linkName,
                                              const gz::math::Vector3d& _linearVelocity)
{
    Entity entity = _ecm.EntityByComponents(components::Name("link_robocomp"));

    Link link(entity);

    link.SetLinearVelocity(_ecm, _linearVelocity);
    optional<string> name = link.Name(_ecm);
    cout << "NOMBRE: " << name.value() << endl;

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

//////////////////////////////////////////////////
void EntitiesControl::Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &/*_eventMgr*/)
{
    this->dataPtr->model = Model(_entity);
    this->dataPtr->sdfConfig = _sdf->Clone();
}

void EntitiesControl::PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm)
{
    if (!this->dataPtr->initialized)
    {
        // We call Load here instead of Configure because we can't be guaranteed
        // that all entities have been created when Configure is called

        //enableComponent<components::WorldPose>(_ecm, this->dataPtr->link.Entity());
        this->dataPtr->initialized = true;
    }

    this->dataPtr->Load(_ecm, this->dataPtr->sdfConfig);
}

//////////////////////////////////////////////////
// Here we implement the PostUpdate function, which is called at every
// iteration.
void EntitiesControl::PostUpdate(const gz::sim::UpdateInfo &_info,
                            const gz::sim::EntityComponentManager &/*_ecm*/)
{
    /*
    // This is a simple example of how to get information from UpdateInfo.
    std::string msg = "Hello, world! Simulation is ";
    if (!_info.paused)
      msg += "not ";
    msg += "paused.";

    // Messages printed with gzmsg only show when running with verbosity 3 or
    // higher (i.e. gz sim -v 3)
    gzmsg << msg << std::endl;
    */
}

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
GZ_ADD_PLUGIN(
        entities_control::EntitiesControl,
        gz::sim::System,
        entities_control::EntitiesControl::ISystemConfigure,
        entities_control::EntitiesControl::ISystemPostUpdate,
        entities_control::EntitiesControl::ISystemPreUpdate
)

