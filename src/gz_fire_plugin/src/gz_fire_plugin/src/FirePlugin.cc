#include <algorithm>
#include <cmath>
#include <cctype>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <gz/common/Console.hh>

#include <gz/math/Color.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/particle_emitter.pb.h>
#include <gz/msgs/visual.pb.h>

#include <gz/transport/Node.hh>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>

#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/VisualCmd.hh>
#include <gz/sim/components/Material.hh>

#include <sdf/Element.hh>
#include <sdf/Material.hh>

#include <gz/plugin/Register.hh>

#include "gz_fire_plugin/FirePlugin.hh"

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{
namespace systems
{

//////////////////////////////////////////////////////////////
// Helper functions
//////////////////////////////////////////////////////////////

namespace
{

double Clamp01(const double _v)
{
  return std::clamp(_v, 0.0, 1.0);
}

double DegToRad(const double _deg)
{
  return _deg * M_PI / 180.0;
}

//////////////////////////////////////////////////////////////
// Create a particle command.
//
// This is the Gazebo Harmonic command format:
//
// emitting
// rate
// velocity
// lifetime
// particle_size
// color_start
// color_end
//////////////////////////////////////////////////////////////

msgs::ParticleEmitter BuildParticleCommand(
    const std::string &_name,
    const bool _active,
    const double _rate,
    const double _minVelocity,
    const double _maxVelocity,
    const double _lifetime,
    const double _particleSize,
    const math::Color &_startColor,
    const math::Color &_endColor)
{
  msgs::ParticleEmitter msg;

  msg.set_name(_name);

  msg.set_type(msgs::ParticleEmitter::BOX);

  msg.mutable_emitting()->set_data(_active);

  msg.mutable_rate()->set_data(
      std::max(0.0, _rate));

  msg.mutable_min_velocity()->set_data(
      std::max(0.0, _minVelocity));

  msg.mutable_max_velocity()->set_data(
      std::max(_minVelocity, _maxVelocity));

  msg.mutable_lifetime()->set_data(
      std::max(0.05, _lifetime));

  msg.mutable_scale_rate()->set_data(0.7);

  msgs::Set(
      msg.mutable_size(),
      math::Vector3d(
          0.035,
          0.035,
          0.035));

  msgs::Set(
      msg.mutable_particle_size(),
      math::Vector3d(
          _particleSize,
          _particleSize,
          _particleSize));

  msgs::Set(
      msg.mutable_color_start(),
      _startColor);

  msgs::Set(
      msg.mutable_color_end(),
      _endColor);

  return msg;
}

}  // anonymous namespace


//////////////////////////////////////////////////////////////
// Private implementation
//////////////////////////////////////////////////////////////

class FirePluginPrivate
{
public:

  ////////////////////////////////////////////////////////////
  // Configuration
  ////////////////////////////////////////////////////////////

  Entity modelEntity{kNullEntity};

  Entity fireLinkEntity{kNullEntity};

  std::string fireLinkName{"fire_source_link"};

  std::string fireTopic{"/fire/trigger"};

  std::string stateTopic{"/fire/state"};

  std::string fireEmitterTopic;

  std::string smokeEmitterTopic;


  ////////////////////////////////////////////////////////////
  // Fire geometry
  ////////////////////////////////////////////////////////////

  double coneHalfAngleDeg{20.0};

  double coneMaxRange{2.0};


  ////////////////////////////////////////////////////////////
  // Fire parameters
  ////////////////////////////////////////////////////////////

  double intensity{1.0};

  double particleRate{200.0};

  bool enableParticles{true};

  bool startOn{true};


  ////////////////////////////////////////////////////////////
  // Burn model
  ////////////////////////////////////////////////////////////

  double ignitionHeat{1.0};

  double heatRate{1.0};

  double damageRate{0.25};

  double destroyDamage{10.0};

  bool destroyAfterBurn{false};


  ////////////////////////////////////////////////////////////
  // Ray / burn parameters
  ////////////////////////////////////////////////////////////

  int numRays{20};

  double markSpacing{0.05};

  int paintIntervalSteps{10};


  ////////////////////////////////////////////////////////////
  // Runtime state
  ////////////////////////////////////////////////////////////

  bool configured{false};

  bool fireOn{false};

  bool previousFireState{false};

  int stepCounter{0};


  ////////////////////////////////////////////////////////////
  // Burnable objects
  ////////////////////////////////////////////////////////////

  std::unordered_set<std::string> burnableModels;


  ////////////////////////////////////////////////////////////
  // Per-model burn state
  ////////////////////////////////////////////////////////////

  struct BurnState
  {
    double heat{0.0};

    double damage{0.0};

    bool ignited{false};

    bool destroyed{false};

    double lastExposure{0.0};
  };

  std::unordered_map<Entity, BurnState> burns;


  ////////////////////////////////////////////////////////////
  // Transport
  ////////////////////////////////////////////////////////////

  transport::Node node;

  transport::Node::Publisher statePub;

  transport::Node::Publisher fireEmitterPub;

  transport::Node::Publisher smokeEmitterPub;


  ////////////////////////////////////////////////////////////
  // Helpers
  ////////////////////////////////////////////////////////////

  bool IsBurnable(const std::string &_name) const
  {
    if (this->burnableModels.empty())
      return true;

    return this->burnableModels.count(_name) > 0;
  }


  ////////////////////////////////////////////////////////////
  // Find visual of a model
  ////////////////////////////////////////////////////////////

  ////////////////////////////////////////////////////////////
  // Get every visual belonging to the model.
  //
  // IMPORTANT:
  // The table is one model with several visuals:
  //   - tabletop_visual
  //   - leg_fl_visual
  //   - leg_fr_visual
  //   - leg_bl_visual
  //   - leg_br_visual
  //
  // Burning only the first visual makes only one part of the
  // table look damaged. We therefore return all visuals.
  ////////////////////////////////////////////////////////////

  std::vector<Entity> FindVisuals(
      const Entity _model,
      EntityComponentManager &_ecm) const
  {
    std::vector<Entity> result;

    Model model(_model);

    const auto links = model.Links(_ecm);

    for (const auto linkEntity : links)
    {
      Link link(linkEntity);

      const auto visuals = link.Visuals(_ecm);

      for (const auto visualEntity : visuals)
      {
        if (visualEntity != kNullEntity)
          result.push_back(visualEntity);
      }
    }

    return result;
  }


  ////////////////////////////////////////////////////////////
  // Darken object material progressively.
  //
  // This is intentionally done with VisualCmd.
  // It avoids the old non-existent components::VisualCmd
  // problem caused by a missing include.
  ////////////////////////////////////////////////////////////

  void SetCharredMaterial(
      const Entity _visual,
      const double _fraction,
      EntityComponentManager &_ecm)
  {
    if (_visual == kNullEntity)
      return;

    auto *matComp =
        _ecm.Component<components::Material>(_visual);

    if (!matComp)
      return;

    auto mat = matComp->Data();

    const double f = Clamp01(_fraction);

    //////////////////////////////////////////////////////////
    // Original material
    //////////////////////////////////////////////////////////

    const auto originalDiffuse =
        mat.Diffuse();

    const auto originalAmbient =
        mat.Ambient();

    const auto originalSpecular =
        mat.Specular();


    //////////////////////////////////////////////////////////
    // Burn toward dark charcoal
    //////////////////////////////////////////////////////////

    const double targetR = 0.055;
    const double targetG = 0.035;
    const double targetB = 0.025;

    const double r =
        originalDiffuse.R() * (1.0 - f) +
        targetR * f;

    const double g =
        originalDiffuse.G() * (1.0 - f) +
        targetG * f;

    const double b =
        originalDiffuse.B() * (1.0 - f) +
        targetB * f;


    sdf::Material newMat = mat;

    newMat.SetDiffuse(
        math::Color(
            static_cast<float>(r),
            static_cast<float>(g),
            static_cast<float>(b),
            originalDiffuse.A()));


    //////////////////////////////////////////////////////////
    // Ambient also becomes dark
    //////////////////////////////////////////////////////////

    newMat.SetAmbient(
        math::Color(
            static_cast<float>(
                originalAmbient.R() * (1.0 - f) +
                0.025 * f),

            static_cast<float>(
                originalAmbient.G() * (1.0 - f) +
                0.018 * f),

            static_cast<float>(
                originalAmbient.B() * (1.0 - f) +
                0.012 * f),

            originalAmbient.A()));


    //////////////////////////////////////////////////////////
    // Reduce specular while burning
    //////////////////////////////////////////////////////////

    newMat.SetSpecular(
        math::Color(
            static_cast<float>(
                originalSpecular.R() * (1.0 - f)),

            static_cast<float>(
                originalSpecular.G() * (1.0 - f)),

            static_cast<float>(
                originalSpecular.B() * (1.0 - f)),

            originalSpecular.A()));


    //////////////////////////////////////////////////////////
    // Update Material component
    //////////////////////////////////////////////////////////

    matComp->SetData(newMat,
        [](const sdf::Material &,
           const sdf::Material &)
        {
          return false;
        });


    //////////////////////////////////////////////////////////
    // Also send a VisualCmd so rendering is updated.
    //////////////////////////////////////////////////////////

    auto *visualCmd =
        _ecm.Component<components::VisualCmd>(_visual);

    msgs::Visual visualMsg;

    if (visualCmd)
    {
      visualMsg = visualCmd->Data();
    }

    msgs::Material *msgMat =
        visualMsg.mutable_material();

    msgs::Set(
        msgMat->mutable_diffuse(),
        math::Color(
            static_cast<float>(r),
            static_cast<float>(g),
            static_cast<float>(b),
            originalDiffuse.A()));

    msgs::Set(
        msgMat->mutable_ambient(),
        math::Color(
            static_cast<float>(
                originalAmbient.R() * (1.0 - f) +
                0.025 * f),

            static_cast<float>(
                originalAmbient.G() * (1.0 - f) +
                0.018 * f),

            static_cast<float>(
                originalAmbient.B() * (1.0 - f) +
                0.012 * f),

            originalAmbient.A()));

    _ecm.SetComponentData<components::VisualCmd>(
        _visual,
        visualMsg);
  }


  ////////////////////////////////////////////////////////////
  // Publish flame + smoke command
  ////////////////////////////////////////////////////////////

  void SetParticleEmitterState(
      const bool _on)
  {
    if (!this->enableParticles)
      return;


    //////////////////////////////////////////////////////////
    // FIRE
    //
    // Yellow -> orange -> red
    //////////////////////////////////////////////////////////

    if (this->fireEmitterPub)
    {
      auto fire =
          BuildParticleCommand(
              "fire_emitter",
              _on,

              260.0 * this->intensity,

              0.35,

              1.40,

              0.55,

              0.075,

              math::Color(
                  1.0f,
                  0.85f,
                  0.05f,
                  1.0f),

              math::Color(
                  0.95f,
                  0.05f,
                  0.0f,
                  0.0f));

      this->fireEmitterPub.Publish(fire);
    }


    //////////////////////////////////////////////////////////
    // SMOKE
    //
    // Grey -> transparent grey
    //////////////////////////////////////////////////////////

    if (this->smokeEmitterPub)
    {
      auto smoke =
          BuildParticleCommand(
              "smoke_emitter",
              _on,

              35.0 * this->intensity,

              0.15,

              0.55,

              2.5,

              0.12,

              math::Color(
                  0.20f,
                  0.20f,
                  0.20f,
                  0.65f),

              math::Color(
                  0.08f,
                  0.08f,
                  0.08f,
                  0.0f));

      this->smokeEmitterPub.Publish(smoke);
    }
  }


  ////////////////////////////////////////////////////////////
  // Publish fire state
  ////////////////////////////////////////////////////////////

  void PublishState()
  {
    if (!this->statePub)
      return;

    msgs::Boolean msg;

    msg.set_data(this->fireOn);

    this->statePub.Publish(msg);
  }


  ////////////////////////////////////////////////////////////
  // Fire trigger callback
  ////////////////////////////////////////////////////////////

  void OnFireTrigger(
      const msgs::Boolean &_msg)
  {
    this->fireOn = _msg.data();
  }


  ////////////////////////////////////////////////////////////
  // Calculate exposure between fire source and target
  ////////////////////////////////////////////////////////////

  double ComputeExposure(
      const Entity _target,
      EntityComponentManager &_ecm)
  {
    if (this->fireLinkEntity == kNullEntity)
      return 0.0;

    if (_target == kNullEntity)
      return 0.0;


    //////////////////////////////////////////////////////////
    // Fire source pose
    //////////////////////////////////////////////////////////

    const auto sourcePose =
        worldPose(
            this->fireLinkEntity,
            _ecm);

    const auto source =
        sourcePose.Pos();


    //////////////////////////////////////////////////////////
    // Target model wrapper
    //////////////////////////////////////////////////////////

    Model model(_target);

    auto links =
        model.Links(_ecm);

    if (links.empty())
      return 0.0;


    //////////////////////////////////////////////////////////
    // Target center
    //////////////////////////////////////////////////////////

    math::Vector3d targetCenter(0, 0, 0);

    int validLinks = 0;

    for (const auto linkEntity : links)
    {
      const auto pose =
          worldPose(
              linkEntity,
              _ecm);

      targetCenter += pose.Pos();

      ++validLinks;
    }

    if (validLinks == 0)
      return 0.0;

    targetCenter /= static_cast<double>(validLinks);


    //////////////////////////////////////////////////////////
    // Relative vector
    //////////////////////////////////////////////////////////

    const auto rel =
        targetCenter - source;

    const double distance =
        rel.Length();

    if (distance < 0.05 ||
        distance > this->coneMaxRange)
    {
      return 0.0;
    }


    //////////////////////////////////////////////////////////
    // Fire direction
    //
    // fire_nozzle pose is:
    // 0 -1.5 0.5 0 0 0
    //
    // Local +Y is aimed at target.
    //////////////////////////////////////////////////////////

    math::Vector3d direction(0, 1, 0);


    //////////////////////////////////////////////////////////
    // If source link has a world pose orientation,
    // transform local +Y to world.
    //////////////////////////////////////////////////////////

    const auto pose =
        worldPose(
            this->fireLinkEntity,
            _ecm);

    direction =
        pose.Rot().RotateVector(
            math::Vector3d(0, 1, 0));

    direction.Normalize();


    //////////////////////////////////////////////////////////
    // Alignment
    //////////////////////////////////////////////////////////

    const auto normalizedRel =
        rel / distance;

    const double alignment =
        normalizedRel.Dot(direction);


    const double halfAngle =
        DegToRad(this->coneHalfAngleDeg);

    const double minAlignment =
        std::cos(halfAngle);


    if (alignment < minAlignment)
      return 0.0;


    //////////////////////////////////////////////////////////
    // Distance falloff
    //////////////////////////////////////////////////////////

    const double distanceFactor =
        Clamp01(
            1.0 -
            distance / this->coneMaxRange);


    //////////////////////////////////////////////////////////
    // Angular falloff
    //////////////////////////////////////////////////////////

    const double angleFactor =
        Clamp01(
            (alignment - minAlignment) /
            std::max(
                1e-6,
                1.0 - minAlignment));


    //////////////////////////////////////////////////////////
    // Final exposure
    //////////////////////////////////////////////////////////

    return Clamp01(
        0.7 * distanceFactor +
        0.3 * angleFactor);
  }


  ////////////////////////////////////////////////////////////
  // Apply burning
  ////////////////////////////////////////////////////////////

  void BurnModel(
      const Entity _model,
      const double _exposure,
      const double _dt,
      EntityComponentManager &_ecm)
  {
    if (_model == kNullEntity)
      return;


    auto *nameComp =
        _ecm.Component<components::Name>(_model);

    if (!nameComp)
      return;

    const std::string modelName =
        nameComp->Data();


    //////////////////////////////////////////////////////////
    // Ignore nozzle and ground
    //////////////////////////////////////////////////////////

    if (modelName == "ground_plane" ||
        modelName == "fire_nozzle")
    {
      return;
    }


    //////////////////////////////////////////////////////////
    // Check burnable list
    //////////////////////////////////////////////////////////

    if (!this->IsBurnable(modelName))
      return;


    //////////////////////////////////////////////////////////
    // Get burn state
    //////////////////////////////////////////////////////////

    auto &burn =
        this->burns[_model];


    burn.lastExposure =
        _exposure;


    //////////////////////////////////////////////////////////
    // Heat accumulation
    //////////////////////////////////////////////////////////

    burn.heat +=
        this->heatRate *
        this->intensity *
        _exposure *
        _dt;


    //////////////////////////////////////////////////////////
    // Ignition
    //////////////////////////////////////////////////////////

    if (!burn.ignited &&
        burn.heat >= this->ignitionHeat)
    {
      burn.ignited = true;

      gzmsg
          << "[FirePlugin] IGNITION: "
          << modelName
          << "\n";
    }


    if (!burn.ignited)
      return;


    //////////////////////////////////////////////////////////
    // Damage
    //////////////////////////////////////////////////////////

    burn.damage +=
        this->damageRate *
        this->intensity *
        _exposure *
        _dt;


    //////////////////////////////////////////////////////////
    // Burn fraction
    //////////////////////////////////////////////////////////

    const double fraction =
        Clamp01(
            burn.damage /
            std::max(
                0.1,
                this->destroyDamage));


    //////////////////////////////////////////////////////////
    // Char ALL visuals of the burnable model
    //
    // This makes the complete table burn:
    // tabletop + all four legs.
    //////////////////////////////////////////////////////////

    const auto visuals =
        this->FindVisuals(
            _model,
            _ecm);

    for (const auto visual : visuals)
    {
      this->SetCharredMaterial(
          visual,
          fraction,
          _ecm);
    }


    //////////////////////////////////////////////////////////
    // Fully burned
    //////////////////////////////////////////////////////////

    if (fraction >= 1.0 &&
        !burn.destroyed)
    {
      gzmsg
          << "[FirePlugin] FULLY BURNED: "
          << modelName
          << "\n";

      if (this->destroyAfterBurn)
      {
        burn.destroyed = true;

        _ecm.RequestRemoveEntity(
            _model,
            true);
      }
    }
  }
};


//////////////////////////////////////////////////////////////
// FirePlugin constructor
//////////////////////////////////////////////////////////////

FirePlugin::FirePlugin()
  : dataPtr(
      std::make_unique<FirePluginPrivate>())
{
}


//////////////////////////////////////////////////////////////
// FirePlugin destructor
//////////////////////////////////////////////////////////////

FirePlugin::~FirePlugin()
{
}


//////////////////////////////////////////////////////////////
// Configure
//////////////////////////////////////////////////////////////

void FirePlugin::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &)
{
  auto &d = *this->dataPtr;

  d.modelEntity = _entity;


  ////////////////////////////////////////////////////////////
  // fire_link
  ////////////////////////////////////////////////////////////

  if (_sdf->HasElement("fire_link"))
  {
    d.fireLinkName =
        _sdf->Get<std::string>(
            "fire_link");
  }


  ////////////////////////////////////////////////////////////
  // cone
  ////////////////////////////////////////////////////////////

  if (_sdf->HasElement("cone_half_angle"))
  {
    d.coneHalfAngleDeg =
        _sdf->Get<double>(
            "cone_half_angle");
  }

  if (_sdf->HasElement("cone_max_range"))
  {
    d.coneMaxRange =
        _sdf->Get<double>(
            "cone_max_range");
  }


  ////////////////////////////////////////////////////////////
  // Fire topic
  ////////////////////////////////////////////////////////////

  if (_sdf->HasElement("fire_topic"))
  {
    d.fireTopic =
        _sdf->Get<std::string>(
            "fire_topic");
  }


  ////////////////////////////////////////////////////////////
  // State topic
  ////////////////////////////////////////////////////////////

  if (_sdf->HasElement("state_topic"))
  {
    d.stateTopic =
        _sdf->Get<std::string>(
            "state_topic");
  }


  ////////////////////////////////////////////////////////////
  // Particle rate
  ////////////////////////////////////////////////////////////

  if (_sdf->HasElement("particle_rate"))
  {
    d.particleRate =
        _sdf->Get<double>(
            "particle_rate");
  }


  ////////////////////////////////////////////////////////////
  // Enable particles
  ////////////////////////////////////////////////////////////

  if (_sdf->HasElement("enable_particles"))
  {
    d.enableParticles =
        _sdf->Get<bool>(
            "enable_particles");
  }


  ////////////////////////////////////////////////////////////
  // Intensity
  ////////////////////////////////////////////////////////////

  if (_sdf->HasElement("intensity"))
  {
    d.intensity =
        std::clamp(
            _sdf->Get<double>("intensity"),
            0.0,
            5.0);
  }


  ////////////////////////////////////////////////////////////
  // Ray parameters
  ////////////////////////////////////////////////////////////

  if (_sdf->HasElement("num_rays"))
  {
    d.numRays =
        _sdf->Get<int>("num_rays");
  }

  if (_sdf->HasElement("mark_spacing"))
  {
    d.markSpacing =
        _sdf->Get<double>("mark_spacing");
  }

  if (_sdf->HasElement("paint_interval_steps"))
  {
    d.paintIntervalSteps =
        _sdf->Get<int>(
            "paint_interval_steps");
  }


  ////////////////////////////////////////////////////////////
  // Start state
  ////////////////////////////////////////////////////////////

  if (_sdf->HasElement("start_on"))
  {
    d.startOn =
        _sdf->Get<bool>(
            "start_on");
  }


  ////////////////////////////////////////////////////////////
  // Burn parameters
  ////////////////////////////////////////////////////////////

  if (_sdf->HasElement("ignition_heat"))
  {
    d.ignitionHeat =
        std::max(
            0.01,
            _sdf->Get<double>(
                "ignition_heat"));
  }

  if (_sdf->HasElement("heat_rate"))
  {
    d.heatRate =
        std::max(
            0.0,
            _sdf->Get<double>(
                "heat_rate"));
  }

  if (_sdf->HasElement("damage_rate"))
  {
    d.damageRate =
        std::max(
            0.0,
            _sdf->Get<double>(
                "damage_rate"));
  }

  if (_sdf->HasElement("destroy_damage"))
  {
    d.destroyDamage =
        std::max(
            0.1,
            _sdf->Get<double>(
                "destroy_damage"));
  }

  if (_sdf->HasElement("destroy_after_burn"))
  {
    d.destroyAfterBurn =
        _sdf->Get<bool>(
            "destroy_after_burn");
  }


  ////////////////////////////////////////////////////////////
  // Burnable models
  ////////////////////////////////////////////////////////////

  if (_sdf->HasElement("burnable_models"))
  {
    const auto list =
        _sdf->Get<std::string>(
            "burnable_models");

    std::stringstream ss(list);

    std::string item;

    while (std::getline(
        ss,
        item,
        ','))
    {
      item.erase(
          std::remove_if(
              item.begin(),
              item.end(),
              [](unsigned char c)
              {
                return std::isspace(c);
              }),
          item.end());

      if (!item.empty())
        d.burnableModels.insert(item);
    }
  }


  ////////////////////////////////////////////////////////////
  // Clamp
  ////////////////////////////////////////////////////////////

  d.coneMaxRange =
      std::max(
          0.05,
          d.coneMaxRange);

  d.paintIntervalSteps =
      std::max(
          1,
          d.paintIntervalSteps);


  ////////////////////////////////////////////////////////////
  // Find fire link
  ////////////////////////////////////////////////////////////

  Model model(_entity);

  d.fireLinkEntity =
      model.LinkByName(
          _ecm,
          d.fireLinkName);


  if (d.fireLinkEntity == kNullEntity)
  {
    gzerr
        << "[FirePlugin] ERROR: Link '"
        << d.fireLinkName
        << "' not found.\n";

    return;
  }


  ////////////////////////////////////////////////////////////
  // Fire trigger subscriber
  ////////////////////////////////////////////////////////////

  d.node.Subscribe(
      d.fireTopic,
      &FirePluginPrivate::OnFireTrigger,
      &d);


  ////////////////////////////////////////////////////////////
  // State publisher
  ////////////////////////////////////////////////////////////

  d.statePub =
      d.node.Advertise<msgs::Boolean>(
          d.stateTopic);


  ////////////////////////////////////////////////////////////
  // Particle emitter topics
  //
  // If the SDF has explicit topics they are used.
  // Otherwise Harmonic's default naming is used.
  ////////////////////////////////////////////////////////////

  const std::string modelName =
      model.Name(_ecm);

  const std::string base =
      "/model/" +
      modelName +
      "/link/" +
      d.fireLinkName +
      "/particle_emitter/";


  if (_sdf->HasElement("fire_particle_topic"))
  {
    d.fireEmitterTopic =
        _sdf->Get<std::string>(
            "fire_particle_topic");
  }
  else
  {
    d.fireEmitterTopic =
        base +
        "fire_emitter/cmd";
  }


  if (_sdf->HasElement("smoke_particle_topic"))
  {
    d.smokeEmitterTopic =
        _sdf->Get<std::string>(
            "smoke_particle_topic");
  }
  else
  {
    d.smokeEmitterTopic =
        base +
        "smoke_emitter/cmd";
  }


  ////////////////////////////////////////////////////////////
  // Particle publishers
  ////////////////////////////////////////////////////////////

  if (d.enableParticles)
  {
    d.fireEmitterPub =
        d.node.Advertise<msgs::ParticleEmitter>(
            d.fireEmitterTopic);

    d.smokeEmitterPub =
        d.node.Advertise<msgs::ParticleEmitter>(
            d.smokeEmitterTopic);
  }


  ////////////////////////////////////////////////////////////
  // Initial state
  ////////////////////////////////////////////////////////////

  d.fireOn =
      d.startOn;

  d.previousFireState =
      !d.startOn;

  d.configured =
      true;


  ////////////////////////////////////////////////////////////
  // Initial particle state
  ////////////////////////////////////////////////////////////

  d.SetParticleEmitterState(
      d.fireOn);

  d.PublishState();


  ////////////////////////////////////////////////////////////
  // Log
  ////////////////////////////////////////////////////////////

  gzmsg
      << "\n"
      << "========================================\n"
      << "[FirePlugin] READY\n"
      << "  fire link       : "
      << d.fireLinkName
      << "\n"
      << "  fire topic      : "
      << d.fireTopic
      << "\n"
      << "  fire emitter    : "
      << d.fireEmitterTopic
      << "\n"
      << "  smoke emitter   : "
      << d.smokeEmitterTopic
      << "\n"
      << "  cone angle      : "
      << d.coneHalfAngleDeg
      << " deg\n"
      << "  cone range      : "
      << d.coneMaxRange
      << " m\n"
      << "  intensity       : "
      << d.intensity
      << "\n"
      << "  particles       : "
      << (d.enableParticles ? "ON" : "OFF")
      << "\n"
      << "========================================\n";
}


//////////////////////////////////////////////////////////////
// PreUpdate
//////////////////////////////////////////////////////////////

void FirePlugin::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  auto &d = *this->dataPtr;


  if (!d.configured)
    return;

  if (_info.paused)
    return;


  ////////////////////////////////////////////////////////////
  // dt
  ////////////////////////////////////////////////////////////

  double dt = 0.001;

  if (_info.dt.count() > 0)
  {
    dt =
        std::chrono::duration<double>(
            _info.dt).count();
  }

  dt =
      std::clamp(
          dt,
          1e-5,
          0.1);


  ////////////////////////////////////////////////////////////
  // Fire state changed
  ////////////////////////////////////////////////////////////

  if (d.fireOn != d.previousFireState)
  {
    d.SetParticleEmitterState(
        d.fireOn);

    d.PublishState();

    d.previousFireState =
        d.fireOn;


    gzmsg
        << "[FirePlugin] Fire "
        << (d.fireOn ? "ON" : "OFF")
        << "\n";
  }


  ////////////////////////////////////////////////////////////
  // If fire OFF, nothing burns
  ////////////////////////////////////////////////////////////

  if (!d.fireOn)
    return;


  ////////////////////////////////////////////////////////////
  // Only calculate burn every N steps
  ////////////////////////////////////////////////////////////

  ++d.stepCounter;

  if (d.stepCounter %
          d.paintIntervalSteps != 0)
  {
    return;
  }


  ////////////////////////////////////////////////////////////
  // Find all models
  ////////////////////////////////////////////////////////////

  _ecm.Each<components::Model,
            components::Name>(
      [&](const Entity _entity,
          const components::Model *,
          const components::Name *_name)
      {
        if (!_name)
          return true;


        const std::string modelName =
            _name->Data();


        //////////////////////////////////////////////////////
        // Ignore fire source
        //////////////////////////////////////////////////////

        if (_entity == d.modelEntity ||
            modelName == "fire_nozzle" ||
            modelName == "ground_plane")
        {
          return true;
        }


        //////////////////////////////////////////////////////
        // Burnable filtering
        //////////////////////////////////////////////////////

        if (!d.IsBurnable(modelName))
          return true;


        //////////////////////////////////////////////////////
        // Exposure
        //////////////////////////////////////////////////////

        const double exposure =
            d.ComputeExposure(
                _entity,
                _ecm);


        if (exposure <= 0.0)
          return true;


        //////////////////////////////////////////////////////
        // Burn
        //////////////////////////////////////////////////////

        d.BurnModel(
            _entity,
            exposure,
            dt *
                static_cast<double>(
                    d.paintIntervalSteps),
            _ecm);


        return true;
      });
}


//////////////////////////////////////////////////////////////
// Plugin registration
//////////////////////////////////////////////////////////////

}  // namespace systems
}  // inline namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz


GZ_ADD_PLUGIN(
    gz::sim::systems::FirePlugin,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::FirePlugin,
    "gz::sim::systems::FirePlugin")