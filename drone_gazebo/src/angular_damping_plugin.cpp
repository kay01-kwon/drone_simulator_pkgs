#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Vector3.hh>

namespace angular_damping
{

class AngularDamping
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &) override
  {
    this->model_ = gz::sim::Model(_entity);

    if (_sdf->HasElement("link_name"))
      this->linkName_ = _sdf->Get<std::string>("link_name");
    if (_sdf->HasElement("b_x"))
      this->bx_ = _sdf->Get<double>("b_x");
    if (_sdf->HasElement("b_y"))
      this->by_ = _sdf->Get<double>("b_y");
    if (_sdf->HasElement("b_z"))
      this->bz_ = _sdf->Get<double>("b_z");

    gzmsg << "[AngularDamping] link=" << linkName_
          << "  b=[" << bx_ << ", " << by_ << ", " << bz_
          << "] Nm·s/rad" << std::endl;
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;

    if (!this->linkInitialized_)
    {
      auto entity = this->model_.LinkByName(_ecm, this->linkName_);
      if (entity == gz::sim::kNullEntity)
        return;
      this->link_ = gz::sim::Link(entity);
      this->link_.EnableVelocityChecks(_ecm);
      this->linkInitialized_ = true;
    }

    auto worldAngVel = this->link_.WorldAngularVelocity(_ecm);
    auto worldPose = this->link_.WorldPose(_ecm);
    if (!worldAngVel || !worldPose)
      return;

    auto rot = worldPose->Rot();
    auto bodyAngVel = rot.Inverse().RotateVector(*worldAngVel);

    gz::math::Vector3d torqueBody(
        -this->bx_ * bodyAngVel.X(),
        -this->by_ * bodyAngVel.Y(),
        -this->bz_ * bodyAngVel.Z());

    auto torqueWorld = rot.RotateVector(torqueBody);

    this->link_.AddWorldWrench(_ecm,
        gz::math::Vector3d::Zero,
        torqueWorld);
  }

private:
  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Link link_{gz::sim::kNullEntity};
  bool linkInitialized_{false};
  std::string linkName_{"base_link"};
  double bx_{0.0};
  double by_{0.0};
  double bz_{0.0};
};

}  // namespace angular_damping

GZ_ADD_PLUGIN(angular_damping::AngularDamping,
              gz::sim::System,
              angular_damping::AngularDamping::ISystemConfigure,
              angular_damping::AngularDamping::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(angular_damping::AngularDamping,
                    "angular_damping::AngularDamping")
