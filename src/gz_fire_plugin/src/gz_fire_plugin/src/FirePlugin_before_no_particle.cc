// Copyright 2024 — Apache-2.0 License
// gz_fire_plugin — Gazebo Harmonic + ROS 2 Jazzy
//
// Fire + burn interaction:
//   * Native Gazebo ParticleEmitter systems provide the flame.
//   * /fire/trigger toggles the flame.
//   * The plugin detects burnable model links inside the flame cone using
//     their collision-derived axis-aligned bounding boxes.
//   * Heat accumulates while a burnable object is exposed.
//   * After ignition, the object's visual material progressively chars.
//   * No dynamic ECS entities are created, avoiding the previous graph error.
//
// This is a simulation burn model, not a thermodynamically complete combustion
// solver. It provides exposure -> heat -> ignition -> damage -> char/destruction
// behavior suitable for a Gazebo fire interaction demo.

#include "gz_fire_plugin/FirePlugin.hh"

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <gz/math/AxisAlignedBox.hh>
#include <gz/math/Color.hh>
#include <gz/math/Pose3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/particle_emitter.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AxisAlignedBox.hh>
#include <gz/sim/components/Material.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/transport/Node.hh>
#include <sdf/sdf.hh>

using namespace gz;
using namespace gz::sim;
using namespace gz::sim::systems;

namespace
{
double Clamp01(const double v)
{
  return std::clamp(v, 0.0, 1.0);
}

double PointAabbDistance(
    const math::Vector3d &_p,
    const math::AxisAlignedBox &_box)
{
  const auto min = _box.Min();
  const auto max = _box.Max();

  double d2 = 0.0;
  for (int i = 0; i < 3; ++i)
  {
    const double p = _p[i];
    if (p < min[i])
      d2 += (min[i] - p) * (min[i] - p);
    else if (p > max[i])
      d2 += (p - max[i]) * (p - max[i]);
  }
  return std::sqrt(d2);
}

struct BurnState
{
  double heat{0.0};
  double damage{0.0};
  bool ignited{false};
  bool destroyed{false};
  std::unordered_map<Entity, sdf::Material> originalMaterials;
};
}

class gz::sim::systems::FirePluginPrivate
{
public:
  std::string fireLinkName{"fire_source_link"};
  double coneHalfAngleDeg{20.0};
  double coneMaxRange{2.0};
  std::string fireTopic{"/fire/trigger"};

  std::string coreParticleTopic;
  std::string innerParticleTopic;
  std::string outerParticleTopic;

  double particleRate{200.0};
  int numRays{20};
  double intensity{1.0};
  math::Color fireColor{1.0f, 0.4f, 0.0f, 1.0f};
  int paintIntervalSteps{10};
  bool enableParticles{true};
  bool startOn{true};

  // Burn model parameters.
  double ignitionHeat{1.0};
  double heatRate{0.75};
  double damageRate{0.08};
  double destroyDamage{100.0};
  bool destroyAfterBurn{false};
  std::unordered_set<std::string> burnableModels;

  Entity modelEntity{kNullEntity};
  Entity fireLinkEntity{kNullEntity};

  bool fireOn{false};
  bool fireOnPrev{false};
  bool configured{false};
  std::mutex fireMutex;

  int stepCount{0};

  transport::Node node;
  transport::Node::Publisher statePub;
  transport::Node::Publisher coreParticlePub;
  transport::Node::Publisher innerParticlePub;
  transport::Node::Publisher outerParticlePub;

  std::unordered_map<Entity, BurnState> burns;

  void OnFireTrigger(const msgs::Boolean &_msg)
  {
    std::lock_guard<std::mutex> lock(this->fireMutex);
    this->fireOn = _msg.data();
  }

  bool IsBurnable(const std::string &_name) const
  {
    return this->burnableModels.empty() ||
           this->burnableModels.count(_name) > 0;
  }

  msgs::ParticleEmitter BuildParticleCommand(
      const std::string &_name,
      bool _active,
      double _rate,
      double _minVel,
      double _maxVel,
      double _lifetime,
      double _particleSize,
      const math::Color &_startColor,
      const math::Color &_endColor) const
  {
    msgs::ParticleEmitter emitter;
    emitter.set_name(_name);
    emitter.set_type(msgs::ParticleEmitter::BOX);
    emitter.mutable_emitting()->set_data(_active);
    emitter.mutable_rate()->set_data(std::max(0.0, _rate));
    emitter.mutable_min_velocity()->set_data(std::max(0.0, _minVel));
    emitter.mutable_max_velocity()->set_data(
        std::max(_minVel, _maxVel));
    emitter.mutable_lifetime()->set_data(std::max(0.05, _lifetime));

    msgs::Set(emitter.mutable_size(),
              math::Vector3d(0.04, 0.04, 0.04));
    msgs::Set(emitter.mutable_particle_size(),
              math::Vector3d(_particleSize,
                             _particleSize,
                             _particleSize));
    msgs::Set(emitter.mutable_color_start(), _startColor);
    msgs::Set(emitter.mutable_color_end(), _endColor);
    return emitter;
  }

  void SetCharredMaterial(
      const Entity _visual,
      const double _fraction,
      EntityComponentManager &_ecm)
  {
    auto *matComp = _ecm.Component<components::Material>(_visual);
    if (!matComp)
      return;

    auto mat = matComp->Data();
    const double f = Clamp01(_fraction);

    const auto ambient = mat.Ambient();
    const auto diffuse = mat.Diffuse();

    const float ar = static_cast<float>(
        ambient.R() * (1.0 - f) + 0.025 * f);
    const float ag = static_cast<float>(
        ambient.G() * (1.0 - f) + 0.012 * f);
    const float ab = static_cast<float>(
        ambient.B() * (1.0 - f) + 0.008 * f);

    const float dr = static_cast<float>(
        diffuse.R() * (1.0 - f) + 0.04 * f);
    const float dg = static_cast<float>(
        diffuse.G() * (1.0 - f) + 0.018 * f);
    const float db = static_cast<float>(
        diffuse.B() * (1.0 - f) + 0.012 * f);

    mat.SetAmbient(math::Color(ar, ag, ab, ambient.A()));
    mat.SetDiffuse(math::Color(dr, dg, db, diffuse.A()));

    const float glow = static_cast<float>(
        (f < 0.95) ? (1.0 - f) * 0.12 : 0.0);

    mat.SetEmissive(
        math::Color(glow, glow * 0.08f, 0.0f, 1.0f));

    matComp->SetData(
        mat,
        [](const sdf::Material &, const sdf::Material &)
        {
          return true;
        });

    _ecm.SetChanged(
        _visual,
        components::Material::typeId,
        ComponentState::OneTimeChange);
  }

  void CharModel(
      const Entity _model,
      BurnState &_burn,
      const double _fraction,
      EntityComponentManager &_ecm)
  {
    Model model(_model);

    for (const auto linkEntity : model.Links(_ecm))
    {
      const auto visuals =
          _ecm.ChildrenByComponents(
              linkEntity,
              components::Visual());

      for (const auto visual : visuals)
      {
        this->SetCharredMaterial(visual, _fraction, _ecm);
      }
    }
  }


  void UpdateBurning(EntityComponentManager &_ecm, const double _dt)
  {
    if (!this->fireOn || this->fireLinkEntity == kNullEntity)
      return;

    const auto firePose = worldPose(this->fireLinkEntity, _ecm);

    // The SDF emitter rotates local +X to world +Y, so use local +Y for
    // the physical flame direction as well.
    const math::Vector3d direction =
        firePose.Rot().RotateVector(math::Vector3d::UnitY).Normalized();
    const math::Vector3d origin =
        firePose.Pos() + direction * 0.10;

    const double halfRad =
        std::clamp(this->coneHalfAngleDeg, 0.0, 89.9) *
        3.14159265358979323846 / 180.0;
    const double cosHalf = std::cos(halfRad);

    _ecm.Each<components::Model, components::Name>(
        [&](const Entity &_model,
            const components::Model *,
            const components::Name *_name) -> bool
        {
          if (_model == this->modelEntity ||
              _model == kNullEntity ||
              !_name)
            return true;

          const std::string modelName = _name->Data();

          if (modelName == "ground_plane" ||
              modelName == "fire_nozzle" ||
              !this->IsBurnable(modelName))
            return true;

          Model model(_model);
          double bestExposure = 0.0;

          for (const auto linkEntity : model.Links(_ecm))
          {
            Link link(linkEntity);
            auto localBox = link.ComputeAxisAlignedBox(_ecm);
            if (!localBox.has_value() || localBox->Size().Length() <= 1e-9)
              continue;

            const auto pose = worldPose(linkEntity, _ecm);
            const auto worldBox =
                transformAxisAlignedBox(*localBox, pose);

            const auto center = worldBox.Center();
            const auto rel = center - origin;
            const double distance = rel.Length();

            if (distance < 0.05 || distance > this->coneMaxRange)
              continue;

            const double alignment =
                rel.Normalized().Dot(direction);

            // Account for the physical radius of the bounding box so that
            // an object intersecting the cone edge can still ignite.
            const double radius =
                0.5 * worldBox.Size().Length();

            const double angularMargin =
                std::asin(std::clamp(
                    radius / std::max(distance, radius + 1e-6),
                    0.0, 0.999));

            const double effectiveCos =
                std::cos(std::max(0.0, halfRad - angularMargin));

            if (alignment < effectiveCos)
              continue;

            const double distanceFactor =
                Clamp01(1.0 - distance / this->coneMaxRange);

            const double angleFactor =
                Clamp01((alignment - effectiveCos) /
                        std::max(1e-6, 1.0 - effectiveCos));

            const double exposure =
                std::max(distanceFactor * 0.7 +
                         angleFactor * 0.3, 0.0);

            bestExposure = std::max(bestExposure, exposure);
          }

          if (bestExposure <= 0.0)
            return true;

          auto &burn = this->burns[_model];

          const double heatGain =
              this->heatRate * this->intensity *
              bestExposure * _dt;

          burn.heat += heatGain;

          if (!burn.ignited && burn.heat >= this->ignitionHeat)
          {
            burn.ignited = true;
            gzmsg << "[FirePlugin] IGNITION: " << modelName
                  << " heat=" << burn.heat << "\n";
          }

          if (burn.ignited)
          {
            burn.damage +=
                this->damageRate * this->intensity *
                bestExposure * _dt;

            const double charFraction =
                Clamp01(burn.damage / this->destroyDamage);

            this->CharModel(_model, burn, charFraction, _ecm);

            if (charFraction >= 1.0)
            {
              gzmsg << "[FirePlugin] FULLY BURNED: "
                    << modelName << "\n";

              if (this->destroyAfterBurn && !burn.destroyed)
              {
                burn.destroyed = true;
                _ecm.RequestRemoveEntity(_model, true);
              }
            }
          }

          return true;
        });
  }
};

FirePlugin::FirePlugin()
    : dataPtr(std::make_unique<FirePluginPrivate>())
{
}

FirePlugin::~FirePlugin() = default;

void FirePlugin::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager & /*_eventMgr*/)
{
  auto &d = *this->dataPtr;
  d.modelEntity = _entity;

  if (_sdf->HasElement("fire_link"))
    d.fireLinkName = _sdf->Get<std::string>("fire_link");
  if (_sdf->HasElement("cone_half_angle"))
    d.coneHalfAngleDeg = _sdf->Get<double>("cone_half_angle");
  if (_sdf->HasElement("cone_max_range"))
    d.coneMaxRange = _sdf->Get<double>("cone_max_range");
  if (_sdf->HasElement("fire_topic"))
    d.fireTopic = _sdf->Get<std::string>("fire_topic");
  if (_sdf->HasElement("core_particle_topic"))
    d.coreParticleTopic = _sdf->Get<std::string>("core_particle_topic");
  if (_sdf->HasElement("inner_particle_topic"))
    d.innerParticleTopic = _sdf->Get<std::string>("inner_particle_topic");
  if (_sdf->HasElement("outer_particle_topic"))
    d.outerParticleTopic = _sdf->Get<std::string>("outer_particle_topic");

  if (_sdf->HasElement("particle_rate"))
    d.particleRate = _sdf->Get<double>("particle_rate");
  if (_sdf->HasElement("num_rays"))
    d.numRays = _sdf->Get<int>("num_rays");
  if (_sdf->HasElement("paint_interval_steps"))
    d.paintIntervalSteps = _sdf->Get<int>("paint_interval_steps");
  if (_sdf->HasElement("enable_particles"))
    d.enableParticles = _sdf->Get<bool>("enable_particles");
  if (_sdf->HasElement("start_on"))
    d.startOn = _sdf->Get<bool>("start_on");
  if (_sdf->HasElement("intensity"))
    d.intensity = std::clamp(_sdf->Get<double>("intensity"), 0.0, 2.0);

  if (_sdf->HasElement("ignition_heat"))
    d.ignitionHeat = std::max(
        0.01, _sdf->Get<double>("ignition_heat"));
  if (_sdf->HasElement("heat_rate"))
    d.heatRate = std::max(0.0, _sdf->Get<double>("heat_rate"));
  if (_sdf->HasElement("damage_rate"))
    d.damageRate = std::max(0.0, _sdf->Get<double>("damage_rate"));
  if (_sdf->HasElement("destroy_damage"))
    d.destroyDamage = std::max(
        0.1, _sdf->Get<double>("destroy_damage"));
  if (_sdf->HasElement("destroy_after_burn"))
    d.destroyAfterBurn =
        _sdf->Get<bool>("destroy_after_burn");

  if (_sdf->HasElement("burnable_models"))
  {
    const auto list = _sdf->Get<std::string>("burnable_models");
    std::stringstream ss(list);
    std::string item;
    while (std::getline(ss, item, ','))
    {
      item.erase(
          std::remove_if(item.begin(), item.end(), ::isspace),
          item.end());
      if (!item.empty())
        d.burnableModels.insert(item);
    }
  }

  d.coneMaxRange = std::max(0.0, d.coneMaxRange);
  d.paintIntervalSteps = std::max(1, d.paintIntervalSteps);

  Model model(_entity);
  d.fireLinkEntity = model.LinkByName(_ecm, d.fireLinkName);

  if (d.fireLinkEntity == kNullEntity)
  {
    gzerr << "[FirePlugin] Link '" << d.fireLinkName
          << "' not found.\n";
    return;
  }

  d.node.Subscribe(d.fireTopic,
                   &FirePluginPrivate::OnFireTrigger,
                   &d);

  d.fireOn = d.startOn;
  d.fireOnPrev = !d.startOn;

  if (d.enableParticles)
  {
    const std::string modelName = model.Name(_ecm);
    const std::string base =
        "/model/" + modelName +
        "/link/" + d.fireLinkName +
        "/particle_emitter/";

    if (d.coreParticleTopic.empty())
      d.coreParticleTopic = base + "fire_core/cmd";
    if (d.innerParticleTopic.empty())
      d.innerParticleTopic = base + "fire_inner/cmd";
    if (d.outerParticleTopic.empty())
      d.outerParticleTopic = base + "fire_outer/cmd";

    d.coreParticlePub =
        d.node.Advertise<msgs::ParticleEmitter>(
            d.coreParticleTopic);
    d.innerParticlePub =
        d.node.Advertise<msgs::ParticleEmitter>(
            d.innerParticleTopic);
    d.outerParticlePub =
        d.node.Advertise<msgs::ParticleEmitter>(
            d.outerParticleTopic);
  }

  d.statePub =
      d.node.Advertise<msgs::Boolean>("/fire/state");

  d.configured = true;

  gzmsg << "[FirePlugin] Ready\n"
        << "  burnable models: ";
  for (const auto &name : d.burnableModels)
    gzmsg << name << " ";
  gzmsg << "\n"
        << "  ignition_heat: " << d.ignitionHeat
        << "  heat_rate: " << d.heatRate
        << "  damage_rate: " << d.damageRate << "\n";
}

void FirePlugin::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager & /*_ecm*/)
{
  if (_info.paused)
    return;

  auto &d = *this->dataPtr;
  if (!d.configured)
    return;

  bool currentFireState;
  {
    std::lock_guard<std::mutex> lock(d.fireMutex);
    currentFireState = d.fireOn;
  }

  if (currentFireState == d.fireOnPrev)
    return;

  d.fireOnPrev = currentFireState;

  if (d.enableParticles)
  {
    if (d.coreParticlePub)
      d.coreParticlePub.Publish(
          d.BuildParticleCommand(
              "fire_core", currentFireState,
              150.0 * d.intensity, 0.15, 0.65, 0.32,
              0.055,
              math::Color(1.0f, 0.75f, 0.05f, 0.95f),
              math::Color(1.0f, 0.20f, 0.0f, 0.0f)));

    if (d.innerParticlePub)
      d.innerParticlePub.Publish(
          d.BuildParticleCommand(
              "fire_inner", currentFireState,
              110.0 * d.intensity, 0.30, 1.00, 0.48,
              0.075,
              math::Color(1.0f, 0.42f, 0.02f, 0.90f),
              math::Color(0.85f, 0.05f, 0.0f, 0.0f)));

    if (d.outerParticlePub)
      d.outerParticlePub.Publish(
          d.BuildParticleCommand(
              "fire_outer", currentFireState,
              70.0 * d.intensity, 0.45, 1.25, 0.62,
              0.095,
              math::Color(0.75f, 0.03f, 0.0f, 0.72f),
              math::Color(0.18f, 0.0f, 0.0f, 0.0f)));
  }

  if (d.statePub)
  {
    msgs::Boolean msg;
    msg.set_data(currentFireState);
    d.statePub.Publish(msg);
  }

  gzmsg << "[FirePlugin] Fire "
        << (currentFireState ? "ON" : "OFF") << "\n";
}

void FirePlugin::Update(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  auto &d = *this->dataPtr;
  if (!d.configured || !d.fireOn)
    return;

  const double dt = _info.dt.count();
  if (dt <= 0.0)
    return;

  if (++d.stepCount % d.paintIntervalSteps != 0)
    return;

  d.UpdateBurning(_ecm, dt * d.paintIntervalSteps);
}

GZ_ADD_PLUGIN(
    gz::sim::systems::FirePlugin,
    gz::sim::System,
    gz::sim::systems::FirePlugin::ISystemConfigure,
    gz::sim::systems::FirePlugin::ISystemPreUpdate,
    gz::sim::systems::FirePlugin::ISystemUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::FirePlugin,
    "gz::sim::systems::FirePlugin")