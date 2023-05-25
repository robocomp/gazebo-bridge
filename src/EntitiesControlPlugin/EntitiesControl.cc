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

        Entity targetLink;
        for(auto link : this->iface->ecm->ChildrenByComponents(targetEntity, components::Link())){
            targetLink = link;
        }

        //Construct new message to publish.
        auto msg = msgs::Pose();
        msg.set_name(targetName);

        // Getting the world pose of the link
        gz::math::Pose3d targetPose = gz::sim::worldPose(targetLink, *this->iface->ecm);

        msgs::Vector3d *position = msg.mutable_position();
        position->set_x(targetPose.Pos().X());
        position->set_y(targetPose.Pos().Y());
        position->set_z(targetPose.Pos().Z());


        // Publishing new information to the topic.
        publisher.Publish(msg);

        return true;
    };
};

/// \brief Private EntitiesControl data class.
class gz::sim::systems::EntitiesControlPrivate{
public:

    bool GetWorldPositionService(const msgs::StringMsg &_req,
                           msgs::Boolean &_res);

    bool SetLinkLinearVelocityService(const msgs::Pose &_req,
                                 msgs::Boolean &_res);

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
    

   // Getting the world name
   const components::Name *constCmp = _ecm.Component<components::Name>(_entity);
   const std::string &worldName = constCmp->Data();

   auto validWorldName = transport::TopicUtils::AsValidTopic(worldName);
   if (validWorldName.empty())
   {
       gzerr << "[" << HEADER_NAME << "] "
             << "World name [" << worldName
             << "] doesn't work well with transport, services not advertised."
             << endl;
       return;
   }

    // Creation of get_world_position service
    std::string getWorldPositionService{"/world/" + validWorldName + "/get_world_position"};
    this->dataPtr->node.Advertise(getWorldPositionService,
                                  &EntitiesControlPrivate::GetWorldPositionService, this->dataPtr.get());

    gzmsg << "Get World Position service on [" << getWorldPositionService << "]" << endl;


    // Creation of set_link_linear_velocity service
    std::string setLinkLinearVelocityService{"/world/" + validWorldName + "/set_link_linear_velocity"};
    this->dataPtr->node.Advertise(setLinkLinearVelocityService,
                                  &EntitiesControlPrivate::SetLinkLinearVelocityService, this->dataPtr.get());

    gzmsg << "Set Linear Velocity service on [" << setLinkLinearVelocityService << "]" << endl;


    // Status Message
    gzmsg << "[" << HEADER_NAME << "] Configured." << std::endl;

}

//////////////////////////////////////////////////////////////
void EntitiesControl::PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) {


    // Checking if we have topics to publish. If not, the execution ends here.
    if(this->dataPtr->pendingCmds.empty())
        return;

    // Publishing in topics.
    for (auto &cmd : this->dataPtr->pendingCmds) {
        // Execute
        if (!cmd->Execute())
            continue;
    }

}

//////////////////////////////////////////////////////////////
#pragma endregion Gazebo Execution Flow


bool EntitiesControlPrivate::GetWorldPositionService(const msgs::StringMsg &_req, msgs::Boolean &_res)
{
    Entity entity = this->iface->ecm->EntityByComponents(components::Name(_req.data()));

    // Check if exists an entity with the given parameters.
    if(!entity)
        return false;

    auto topic = ("/model/" + _req.data() + "/get_world_position");

    gz::transport::Node::Publisher getWorldPositionPub = node.Advertise<gz::msgs::Pose>(topic);

    gz::msgs::Pose *msg = new gz::msgs::Pose();

    // We establish the name of the model initially because it's the only thing that doesn't change in runtime.
    msg->set_name(_req.data());

    auto command = std::make_unique<GetGlobalPositionCommand>(msg, this->iface, getWorldPositionPub);
    this->pendingCmds.push_back(std::move(command));

    gzmsg << "[" << HEADER_NAME << "] Topic: " << topic << " created." << std::endl;

    return true;
}

bool EntitiesControlPrivate::SetLinkLinearVelocityService(const msgs::Pose &_req, msgs::Boolean &_res)
{
    Entity entity = this->iface->ecm->EntityByComponents(components::Name(_req.name()), components::Link());

    // Check if exists an entity with the given parameters.
    if (!entity){
        gzerr << "[" << HEADER_NAME << "] Link not found." << endl;
        return false;
    }

    const gz::math::Vector3d& linearVelocity = gz::math::Vector3d(_req.position().x(), _req.position().y(), _req.position().z());

    Link link(entity);
    link.SetLinearVelocity(*this->iface->ecm, linearVelocity);

    return true;
}


// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
GZ_ADD_PLUGIN(
        EntitiesControl,
        System,
        EntitiesControl::ISystemConfigure,
        EntitiesControl::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(EntitiesControl, "gz::sim::systems::EntitiesControl")