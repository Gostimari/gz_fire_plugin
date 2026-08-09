// Copyright 2026
// Apache-2.0
//
// gz_fire_plugin
//
// Gazebo Harmonic + ROS 2 Jazzy
//
// Responsibilities:
//   1. Fire trigger ON/OFF.
//   2. Detect burnable models inside a fire cone.
//   3. Accumulate heat.
//   4. Ignite objects after sufficient heat.
//   5. Accumulate damage.
//   6. Gradually reveal a pre-created burn-mark visual.
//   7. Optionally remove the object after complete burning.
//
// Visual flame and smoke are native Gazebo ParticleEmitter systems
// defined in SDF. This plugin does NOT construct dynamic Visual entities
// and does NOT publish ParticleEmitter protobuf commands.
//
// This avoids the ECS graph problems encountered with dynamically
// creating Visual entities.
//
// NOTE:
// Gazebo Harmonic's Ogre2 renderer has had particle colour-range issues.
// The simulation logic is independent of that renderer issue.

#include "gz_fire_plugin/FirePlugin.hh"

#include <algorithm>
#include <cmath>
#include <memory>
#include <set>
#include <sstream>
#include <string>

#include <gz/common/Console.hh>

#include <gz/math/AxisAlignedBox.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <gz/msgs/boolean.pb.h>

#include <gz/transport/Node.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>

#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParticleEmitter.hh>
#include <gz/sim/components/Transparency.hh>

using namespace gz;
using namespace sim;
using namespace systems;

namespace
{

//////////////////////////////////////////////////
double Clamp01(const double _x)
{
  return std::clamp(_x, 0.0, 1.0);
}

//////////////////////////////////////////////////
double DegToRad(const double _deg)
{
  return _deg * M_PI / 180.0;
}

//////////////////////////////////////////////////
struct BurnState
{
  double heat{0.0};
  double damage{0.0};

  bool ignited{false};
  bool destroyed{false};

  Entity burnMarkVisual{kNullEntity};
};

//////////////////////////////////////////////////
bool IsFinite(const math::Vector3d &_v)
{
  return std::isfinite(_v.X()) &&
         std::isfinite(_v.Y()) &&
         std::isfinite(_v.Z());
}

}  // namespace

//////////////////////////////////////////////////
class FirePluginPrivate
{
  public: Entity modelEntity{kNullEntity};

  public: Entity fireLinkEntity{kNullEntity};

  public: std::string fireLinkName{"fire_source_link"};

  public: std::string fireTopic{"/fire/trigger"};

  public: std::string stateTopic{"/fire/state"};

  public: std::string burnMarkName{"burn_mark"};

  public: double coneHalfAngleDeg{20.0};

  public: double coneMaxRange{2.0};

  public: double intensity{1.2};

  public: double ignitionHeat{1.0};

  public: double heatRate{2.0};

  public: double damageRate{0.35};

  public: double destroyDamage{10.0};

  public: double burnMarkStart{1.0};

  public: double burnMarkFinal{0.12};

  public: bool startOn{true};

  public: bool destroyAfterBurn{false};

  public: std::set<std::string> burnableModels;

  public: bool fireOn{true};

  public: bool configured{false};

  public: transport::Node node;

  public: transport::Node::Publisher statePub;

  public: std::unordered_map<Entity, BurnState> burns;

  public: std::unordered_set<Entity> initializedMarks;

  public: Entity targetModel{kNullEntity};

  public: bool targetSearched{false};

  public:
  void OnFireTrigger(const msgs::Boolean &_msg)
  {
    this->fireOn = _msg.data();

    gzmsg << "[FirePlugin] Fire "
          << (this->fireOn ? "ON" : "OFF")
          << std::endl;
  }

  public:
  void PublishState()
  {
    if (!this->statePub)
      return;

    msgs::Boolean msg;
    msg.set_data(this->fireOn);
    this->statePub.Publish(msg);
  }

  public:
  void SetParticleEmitterState(
      EntityComponentManager &_ecm,
      const std::string &_emitterName)
  {
    _ecm.Each<components::ParticleEmitter,
              components::Name>(
      [&](const Entity _entity,
          const components::ParticleEmitter *_emitter,
          const components::Name *_name)
      {
        if (!_emitter || !_name)
          return true;

        if (_name->Data() != _emitterName)
          return true;

        auto msg = _emitter->Data();

        if (!msg.has_emitting())
          msg.mutable_emitting();

        msg.mutable_emitting()->set_data(this->fireOn);

        _ecm.SetComponentData<components::ParticleEmitter>(
            _entity, msg);

        return true;
      });
  }

  public:
  Entity FindModelByName(
      EntityComponentManager &_ecm,
      const std::string &_name)
  {
    Entity result{kNullEntity};

    _ecm.Each<components::Model,
              components::Name>(
      [&](const Entity _entity,
          const components::Model *,
          const components::Name *_name)
      {
        if (_name && _name->Data() == _name)
        {
          result = _entity;
          return false;
        }

        return true;
      });

    return result;
  }

  public:
  void FindTarget(
      EntityComponentManager &_ecm)
  {
    if (this->targetModel != kNullEntity)
      return;

    for (const auto &name : this->burnableModels)
    {
      _ecm.Each<components::Model,
                components::Name>(
        [&](const Entity _entity,
            const components::Model *,
            const components::Name *_entityName)
        {
          if (_entityName &&
              _entityName->Data() == name)
          {
            this->targetModel = _entity;
            return false;
          }

          return true;
        });

      if (this->targetModel != kNullEntity)
        break;
    }
  }

  public:
  Entity FindBurnMark(
      EntityComponentManager &_ecm,
      const Entity _model)
  {
    if (_model == kNullEntity)
      return kNullEntity;

    Model model(_model);

    for (const auto linkEntity : model.Links(_ecm))
    {
      Link link(linkEntity);

      const Entity visual =
          link.VisualByName(
              _ecm,
              this->burnMarkName);

      if (visual != kNullEntity)
        return visual;
    }

    return kNullEntity;
  }

  public:
  void UpdateBurnMark(
      EntityComponentManager &_ecm,
      BurnState &_burn,
      const double _fraction)
  {
    if (_burn.burnMarkVisual == kNullEntity)
      return;

    const double f = Clamp01(_fraction);

    // 1.0 = invisible
    // burnMarkFinal = strongly visible dark mark.
    const double transparency =
        this->burnMarkStart -
        f * (this->burnMarkStart -
             this->burnMarkFinal);

    _ecm.SetComponentData<components::Transparency>(
        _burn.burnMarkVisual,
        static_cast<float>(
            std::clamp(transparency, 0.0, 1.0)));
  }

  public:
  bool ComputeExposure(
      EntityComponentManager &_ecm,
      const Entity _model,
      const math::Vector3d &_origin,
      const math::Vector3d &_direction,
      double &_exposure)
  {
    _exposure = 0.0;

    if (_model == kNullEntity)
      return false;

    Model model(_model);

    const double halfRad =
        DegToRad(this->coneHalfAngleDeg);

    for (const auto linkEntity : model.Links(_ecm))
    {
      Link link(linkEntity);

      const auto worldBox =
          link.WorldAxisAlignedBox(_ecm);

      if (!worldBox.has_value())
        continue;

      const math::Vector3d center =
          worldBox->Center();

      const math::Vector3d rel =
          center - _origin;

      const double distance =
          rel.Length();

      if (distance < 0.05 ||
          distance > this->coneMaxRange)
      {
        continue;
      }

      if (distance <= 1e-9)
        continue;

      const math::Vector3d relDir =
          rel / distance;

      const double alignment =
          relDir.Dot(_direction);

      const double radius =
          0.5 * worldBox->Size().Length();

      const double angularMargin =
          std::asin(
              std::clamp(
                  radius /
                  std::max(
                      distance,
                      radius + 1e-6),
                  0.0,
                  0.999));

      const double effectiveCos =
          std::cos(
              std::max(
                  0.0,
                  halfRad - angularMargin));

      if (alignment < effectiveCos)
        continue;

      const double distanceFactor =
          Clamp01(
              1.0 -
              distance / this->coneMaxRange);

      const double angleFactor =
          Clamp01(
              (alignment - effectiveCos) /
              std::max(
                  1e-6,
                  1.0 - effectiveCos));

      const double exposure =
          std::max(
              0.7 * distanceFactor +
              0.3 * angleFactor,
              0.0);

      _exposure =
          std::max(
              _exposure,
              exposure);
    }

    return _exposure > 0.0;
  }
};

//////////////////////////////////////////////////
FirePlugin::FirePlugin()
  : dataPtr(std::make_unique<FirePluginPrivate>())
{
}

//////////////////////////////////////////////////
FirePlugin::~FirePlugin() = default;

//////////////////////////////////////////////////
void FirePlugin::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &)
{
  auto &d = *this->dataPtr;

  d.modelEntity = _entity;

  if (_sdf->HasElement("fire_link"))
  {
    d.fireLinkName =
        _sdf->Get<std::string>("fire_link");
  }

  if (_sdf->HasElement("cone_half_angle"))
  {
    d.coneHalfAngleDeg =
        _sdf->Get<double>("cone_half_angle");
  }

  if (_sdf->HasElement("cone_max_range"))
  {
    d.coneMaxRange =
        _sdf->Get<double>("cone_max_range");
  }

  if (_sdf->HasElement("fire_topic"))
  {
    d.fireTopic =
        _sdf->Get<std::string>("fire_topic");
  }

  if (_sdf->HasElement("state_topic"))
  {
    d.stateTopic =
        _sdf->Get<std::string>("state_topic");
  }

  if (_sdf->HasElement("burn_mark_name"))
  {
    d.burnMarkName =
        _sdf->Get<std::string>("burn_mark_name");
  }

  if (_sdf->HasElement("intensity"))
  {
    d.intensity =
        std::clamp(
            _sdf->Get<double>("intensity"),
            0.0,
            5.0);
  }

  if (_sdf->HasElement("ignition_heat"))
  {
    d.ignitionHeat =
        std::max(
            0.01,
            _sdf->Get<double>("ignition_heat"));
  }

  if (_sdf->HasElement("heat_rate"))
  {
    d.heatRate =
        std::max(
            0.0,
            _sdf->Get<double>("heat_rate"));
  }

  if (_sdf->HasElement("damage_rate"))
  {
    d.damageRate =
        std::max(
            0.0,
            _sdf->Get<double>("damage_rate"));
  }

  if (_sdf->HasElement("destroy_damage"))
  {
    d.destroyDamage =
        std::max(
            0.1,
            _sdf->Get<double>("destroy_damage"));
  }

  if (_sdf->HasElement("destroy_after_burn"))
  {
    d.destroyAfterBurn =
        _sdf->Get<bool>(
            "destroy_after_burn");
  }

  if (_sdf->HasElement("start_on"))
  {
    d.startOn =
        _sdf->Get<bool>("start_on");
  }

  if (_sdf->HasElement("burnable_models"))
  {
    const auto list =
        _sdf->Get<std::string>(
            "burnable_models");

    std::stringstream ss(list);
    std::string item;

    while (std::getline(ss, item, ','))
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

  d.coneMaxRange =
      std::max(
          0.0,
          d.coneMaxRange);

  d.fireLinkEntity =
      Model(_entity).LinkByName(
          _ecm,
          d.fireLinkName);

  if (d.fireLinkEntity == kNullEntity)
  {
    gzerr
        << "[FirePlugin] Fire link '"
        << d.fireLinkName
        << "' not found."
        << std::endl;

    return;
  }

  d.node.Subscribe(
      d.fireTopic,
      &FirePluginPrivate::OnFireTrigger,
      &d);

  d.statePub =
      d.node.Advertise<msgs::Boolean>(
          d.stateTopic);

  d.fireOn = d.startOn;

  d.configured = true;

  gzmsg
      << "[FirePlugin] Ready"
      << std::endl
      << "  fire link: "
      << d.fireLinkName
      << std::endl
      << "  cone angle: "
      << d.coneHalfAngleDeg
      << " deg"
      << std::endl
      << "  cone range: "
      << d.coneMaxRange
      << " m"
      << std::endl
      << "  burnable models: ";

  for (const auto &name :
       d.burnableModels)
  {
    gzmsg << name << " ";
  }

  gzmsg
      << std::endl
      << "  fire: "
      << (d.fireOn ? "ON" : "OFF")
      << std::endl;
}

//////////////////////////////////////////////////
void FirePlugin::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  auto &d = *this->dataPtr;

  if (!d.configured ||
      _info.paused)
  {
    return;
  }

  const double dt =
      std::max(
          0.0,
          std::chrono::duration<double>(
              _info.dt).count());

  if (dt <= 0.0)
    return;

  // ----------------------------------------------------------
  // Native emitter ON / OFF
  // ----------------------------------------------------------
  //
  // These components are created by Gazebo's native
  // ParticleEmitter system.
  //
  // No custom ParticleEmitter protobuf publisher is needed.
  //
  d.SetParticleEmitterState(
      _ecm,
      "fire_emitter");

  d.SetParticleEmitterState(
      _ecm,
      "fire_smoke");

  // Publish current state.
  d.PublishState();

  // ----------------------------------------------------------
  // Fire source pose
  // ----------------------------------------------------------

  Link fireLink(d.fireLinkEntity);

  const auto firePose =
      fireLink.WorldPose(_ecm);

  if (!firePose.has_value())
    return;

  const math::Vector3d origin =
      firePose->Pos();

  // Native emitter is rotated so its +X becomes +Y.
  // Use the same direction for heat calculations.
  math::Vector3d direction =
      firePose->Rot() *
      math::Vector3d::UnitY;

  if (!IsFinite(direction) ||
      direction.Length() < 1e-9)
  {
    direction =
        math::Vector3d::UnitY;
  }
  else
  {
    direction.Normalize();
  }

  // ----------------------------------------------------------
  // If fire is OFF, don't accumulate heat.
  // ----------------------------------------------------------

  if (!d.fireOn)
    return;

  // ----------------------------------------------------------
  // Find burnable target models.
  // ----------------------------------------------------------

  std::vector<Entity> targets;

  _ecm.Each<components::Model,
             components::Name>(
      [&](const Entity _entity,
          const components::Model *,
          const components::Name *_name)
      {
        if (!_name)
          return true;

        if (d.burnableModels.count(
                _name->Data()) > 0)
        {
          targets.push_back(_entity);
        }

        return true;
      });

  // ----------------------------------------------------------
  // Process each burnable model.
  // ----------------------------------------------------------

  for (const Entity target :
       targets)
  {
    if (target == d.modelEntity)
      continue;

    double exposure = 0.0;

    if (!d.ComputeExposure(
            _ecm,
            target,
            origin,
            direction,
            exposure))
    {
      continue;
    }

    auto &burn =
        d.burns[target];

    // --------------------------------------------------------
    // Find burn mark visual once the target is fully loaded.
    // --------------------------------------------------------

    if (burn.burnMarkVisual == kNullEntity)
    {
      burn.burnMarkVisual =
          d.FindBurnMark(
              _ecm,
              target);

      if (burn.burnMarkVisual != kNullEntity)
      {
        d.UpdateBurnMark(
            _ecm,
            burn,
            0.0);

        gzmsg
            << "[FirePlugin] Burn mark found for "
            << target
            << std::endl;
      }
    }

    // --------------------------------------------------------
    // Heat accumulation
    // --------------------------------------------------------

    const double heatGain =
        d.heatRate *
        d.intensity *
        exposure *
        dt;

    burn.heat += heatGain;

    // --------------------------------------------------------
    // Ignition
    // --------------------------------------------------------

    if (!burn.ignited &&
        burn.heat >= d.ignitionHeat)
    {
      burn.ignited = true;

      gzmsg
          << "[FirePlugin] IGNITION"
          << " entity=" << target
          << " heat=" << burn.heat
          << std::endl;
    }

    if (!burn.ignited)
      continue;

    // --------------------------------------------------------
    // Damage / charring
    // --------------------------------------------------------

    const double damageGain =
        d.damageRate *
        d.intensity *
        exposure *
        dt;

    burn.damage += damageGain;

    const double charFraction =
        Clamp01(
            burn.damage /
            d.destroyDamage);

    // --------------------------------------------------------
    // Burn mark becomes progressively visible.
    // --------------------------------------------------------

    d.UpdateBurnMark(
        _ecm,
        burn,
        charFraction);

    // --------------------------------------------------------
    // Destroy after complete burn.
    // --------------------------------------------------------

    if (charFraction >= 1.0 &&
        d.destroyAfterBurn &&
        !burn.destroyed)
    {
      burn.destroyed = true;

      gzmsg
          << "[FirePlugin] FULLY BURNED"
          << " entity=" << target
          << std::endl;

      _ecm.RequestRemoveEntity(
          target,
          true);
    }
  }
}

//////////////////////////////////////////////////
GZ_ADD_PLUGIN(
    FirePlugin,
    System,
    FirePlugin::ISystemConfigure,
    FirePlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    FirePlugin,
    "gz::sim::systems::FirePlugin")