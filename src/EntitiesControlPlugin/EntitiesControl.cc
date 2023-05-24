//
// Created by Sergio Eslava Velasco on 2/05/23.
//

// Important to include the plugin's header.
#include "EntitiesControl.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace std;


EntitiesControl::EntitiesControl(): System(), dataPtr(std::make_unique<EntitiesControlPrivate>()){
}

EntitiesControl::~EntitiesControl() = default;


class EntitiesControlInterface{

public:
    /// Pointer to entity component manager. We don't assume ownership.
    EntityComponentManager *ecm{nullptr};

    /// \brief Creator interface, shared among all commands that need it.
    std::unique_ptr<SdfEntityCreator> creator{nullptr};

    /// \brief World entity.
    Entity worldEntity{kNullEntity};

};

class EntitiesControlCommandBase {

public:
    /// \brief Constructor
    /// \param[in] _msg Message containing user command.
    /// \param[in] _iface Pointer to interfaces shared by all commands.
    EntitiesControlCommandBase(google::protobuf::Message *_msg,
    std::shared_ptr<EntitiesControlInterface> &_iface,
    gz::transport::Node::Publisher &_publisher) : msg(_msg), iface(_iface), publisher(_publisher) {};

    /// \brief Destructor.
    virtual ~EntitiesControlCommandBase(){
        if (this->msg != nullptr)
            delete this->msg;
        this->msg = nullptr;
    };

    /// \brief Execute the command. All subclasses must implement this
    /// function and update entities and components so the command takes effect.
    /// \return True if command was properly executed.
    virtual bool Execute() = 0;

protected:

    /// \brief Message containing command.
    google::protobuf::Message *msg{nullptr};

    /// \brief Keep pointer to interfaces shared among commands.
    const std::shared_ptr<EntitiesControlInterface> iface{nullptr};

    /// \brief Publisher instance for the topic
    gz::transport::Node::Publisher publisher;
};

class GetGlobalPositionCommand : public EntitiesControlCommandBase{
public:

    /// \brief Constructor
    /// \param[in] _msg Message identifying the entity to be removed.
    /// \param[in] _iface Pointer to user commands interface.
    GetGlobalPositionCommand(msgs::Pose *_msg,
        std::shared_ptr<EntitiesControlInterface> &_iface,
        gz::transport::Node::Publisher &_publisher) : EntitiesControlCommandBase(_msg, _iface, _publisher) {};

    // Inherited
    bool Execute() final{

        // Casting the general message to the specific one that use this command.
        auto getGlobalPositionMsg = dynamic_cast<const msgs::Pose *>(this->msg);
        if (nullptr == getGlobalPositionMsg)
        {
            gzerr << "Internal error, null create message" << std::endl;
            return false;
        }

        string targetName = getGlobalPositionMsg->name();

        // Get global position from name in the message.
        const Entity targetEntity = this->iface->ecm->EntityByComponents(components::Name(targetName), components::Model());

        Link targetLink;
        for(auto link : this->iface->ecm->ChildrenByComponents(targetEntity, components::Link())){
            targetLink = Link(link);
        }

        //Construct new message to publish.
        auto msg = msgs::Pose();
        msg.set_name(targetName);

        // TODO: Esto devuelve null, no se muy bien por qué.
        // Getting the world pose of the link
        std::optional<gz::math::Pose3d> targetPose = targetLink.WorldPose(*this->iface->ecm);

        if(!targetPose){
            cout << "TargetPose is null" << endl;
        }

        if (targetPose.has_value()) {
            msgs::Vector3d *position = msg.mutable_position();
            position->set_x(targetPose.value().Pos().X());
            position->set_y(targetPose.value().Pos().Y());
            position->set_z(targetPose.value().Pos().Z());
        }else{
            cout  << "No tiene valor" << endl;
        }

        // Publishing new information to the topic.
        publisher.Publish(msg);

        return true;
    };
};

/// \brief Private EntitiesControl data class.
class gz::sim::systems::EntitiesControlPrivate{
public:

    /// Initialize the plugin.
    /// [_ecm] Immutable reference to the EntityComponentManager.
    /// [_sdf] The SDF Element associated with this system plugin.
    void Load(const gz::sim::EntityComponentManager &_ecm);

    /// \brief Flag used for initialization settings in PreUpdate.
    bool initialized{false};

    /// \brief Queue of commands pending execution.
    std::vector<std::unique_ptr<EntitiesControlCommandBase>> pendingCmds;

    /// \brief Gazebo communication node.
    transport::Node node;

    /// \brief Object holding several interfaces that can be used by any command.
    std::shared_ptr<EntitiesControlInterface> iface{nullptr};
};

#pragma region Gazebo Execution Flow

//////////////////////////////////////////////////
void EntitiesControl::Configure(const gz::sim::Entity &_entity,
                                const std::shared_ptr<const sdf::Element> &_sdf,
                                gz::sim::EntityComponentManager &_ecm,
                                gz::sim::EventManager &_eventManager) {

    // Create interfaces shared among commands
    this->dataPtr->iface = std::make_shared<EntitiesControlInterface>();
    this->dataPtr->iface->worldEntity = _entity;
    this->dataPtr->iface->ecm = &_ecm;
    this->dataPtr->iface->creator =
            std::make_unique<SdfEntityCreator>(_ecm, _eventManager);


    // Status Message
    gzmsg << "[" << HEADER_NAME << "] Configured." << std::endl;

}

//////////////////////////////////////////////////////////////
void EntitiesControl::PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) {

    // Initialization settings
    if (!this->dataPtr->initialized) {
        // We call Load here instead of Configure because we can't be guaranteed
        // that all entities have been created when Configure is called
        this->dataPtr->Load(_ecm);

        this->dataPtr->initialized = true;
    }

    // Publishing in topics.
    for (auto &cmd : this->dataPtr->pendingCmds) {
        // Execute
        if (!cmd->Execute())
            continue;
    }


}


// Here we implement the PostUpdate function, which is called at every iteration.
void EntitiesControl::PostUpdate(const gz::sim::UpdateInfo &_info,
                                 const gz::sim::EntityComponentManager &/*_ecm*/) {

}
//////////////////////////////////////////////////////////////
#pragma endregion Gazebo Execution Flow


void EntitiesControlPrivate::Load(const gz::sim::EntityComponentManager &_ecm) {

    ///////////////////////////////////////////////////////
    ////// CREATION OF GetWorldPosition TOPIC /////////////
    ///////////////////////////////////////////////////////

    // Getting all entities in the world scene.
    std::vector<Entity> modelEntities = Model(iface->worldEntity).Models(_ecm);

    for (int i = 0; i < modelEntities.size(); ++i) {

        Model model = Model(modelEntities[i]);

        auto topic = ("/model/" + model.Name(_ecm) + "/get_world_position");

        gz::transport::Node::Publisher getWorldPositionPub = node.Advertise<gz::msgs::Pose>(topic);

        gz::msgs::Pose *msg = new gz::msgs::Pose();

        // We establish the name of the model initially because it's the only thing that doesn't change in runtime.
        msg->set_name(model.Name(_ecm));

        auto command = std::make_unique<GetGlobalPositionCommand>(msg, this->iface, getWorldPositionPub);
        this->pendingCmds.push_back(std::move(command));

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