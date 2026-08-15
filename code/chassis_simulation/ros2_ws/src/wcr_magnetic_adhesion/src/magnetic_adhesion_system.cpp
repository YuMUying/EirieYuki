#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/laserscan.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace wcr::sim
{
class MagneticAdhesionSystem final
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &) override
  {
    this->model_ = gz::sim::Model(_entity);
    if (!this->model_.Valid(_ecm)) {
      gzerr << "MagneticAdhesionSystem must be attached to a model.\n";
      return;
    }

    const std::string linkName =
      _sdf->Get<std::string>("link_name", "base_link").first;
    this->linkEntity_ = this->model_.LinkByName(_ecm, linkName);
    if (this->linkEntity_ == gz::sim::kNullEntity) {
      gzerr << "MagneticAdhesionSystem cannot find link [" << linkName << "].\n";
      return;
    }

    if (!_sdf->HasElement("magnet")) {
      gzerr << "MagneticAdhesionSystem has no <magnet> entries.\n";
      return;
    }

    auto sdfCopy = _sdf->Clone();
    auto magnetElement = sdfCopy->GetElement("magnet");
    while (magnetElement) {
      Magnet magnet;
      magnet.name = magnetElement->Get<std::string>("name", "magnet").first;
      magnet.topic = magnetElement->Get<std::string>("topic", "").first;
      magnet.maxForce = magnetElement->Get<double>("max_force", 0.0).first;
      magnet.fullForceDistance =
        magnetElement->Get<double>("full_force_distance", 0.0).first;
      magnet.releaseDistance =
        magnetElement->Get<double>("release_distance", 0.05).first;
      magnet.exponent = magnetElement->Get<double>("force_exponent", 2.0).first;
      magnet.position = magnetElement->Get<gz::math::Vector3d>(
        "position", gz::math::Vector3d::Zero).first;

      if (magnet.topic.empty() || magnet.maxForce <= 0.0 ||
          magnet.releaseDistance <= magnet.fullForceDistance) {
        gzerr << "Ignoring invalid magnet entry [" << magnet.name << "].\n";
      } else {
        const std::size_t index = this->magnets_.size();
        this->magnets_.push_back(std::move(magnet));
        const bool subscribed = this->transportNode_.Subscribe<gz::msgs::LaserScan>(
          this->magnets_.back().topic,
          [this, index](const gz::msgs::LaserScan &_message)
          {
            this->OnRange(index, _message);
          });
        if (!subscribed) {
          gzerr << "Failed to subscribe to magnet range topic ["
                << this->magnets_.back().topic << "].\n";
        }
      }

      magnetElement = magnetElement->GetNextElement("magnet");
    }

    gzmsg << "Magnetic adhesion configured with " << this->magnets_.size()
          << " attraction points on [" << linkName << "].\n";
  }

  void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || this->linkEntity_ == gz::sim::kNullEntity) {
      return;
    }

    gz::sim::Link link(this->linkEntity_);
    const auto pose = link.WorldPose(_ecm);
    if (!pose.has_value()) {
      return;
    }

    // The sensors look along local -Z. The attraction force uses the same
    // chassis-fixed normal, so it remains correct on floors, ramps and walls.
    const gz::math::Vector3d attractionDirection =
      pose->Rot().RotateVector(gz::math::Vector3d(0.0, 0.0, -1.0));

    std::lock_guard<std::mutex> lock(this->rangeMutex_);
    for (const auto &magnet : this->magnets_) {
      if (!std::isfinite(magnet.range) || magnet.range > magnet.releaseDistance) {
        continue;
      }

      double scale = 1.0;
      if (magnet.range > magnet.fullForceDistance) {
        const double normalized = std::clamp(
          (magnet.releaseDistance - magnet.range) /
          (magnet.releaseDistance - magnet.fullForceDistance),
          0.0, 1.0);
        scale = std::pow(normalized, magnet.exponent);
      }

      link.AddWorldForce(
        _ecm,
        attractionDirection * (magnet.maxForce * scale),
        magnet.position);
    }
  }

private:
  struct Magnet
  {
    std::string name;
    std::string topic;
    double maxForce{0.0};
    double fullForceDistance{0.0};
    double releaseDistance{0.05};
    double exponent{2.0};
    double range{std::numeric_limits<double>::infinity()};
    gz::math::Vector3d position{gz::math::Vector3d::Zero};
  };

  void OnRange(const std::size_t _index, const gz::msgs::LaserScan &_message)
  {
    double minimum = std::numeric_limits<double>::infinity();
    for (int i = 0; i < _message.ranges_size(); ++i) {
      const double value = _message.ranges(i);
      if (std::isfinite(value)) {
        minimum = std::min(minimum, value);
      }
    }

    std::lock_guard<std::mutex> lock(this->rangeMutex_);
    if (_index < this->magnets_.size()) {
      this->magnets_[_index].range = minimum;
    }
  }

  gz::sim::Model model_;
  gz::sim::Entity linkEntity_{gz::sim::kNullEntity};
  gz::transport::Node transportNode_;
  std::vector<Magnet> magnets_;
  std::mutex rangeMutex_;
};
}  // namespace wcr::sim

GZ_ADD_PLUGIN(
  wcr::sim::MagneticAdhesionSystem,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  wcr::sim::MagneticAdhesionSystem,
  "wcr::sim::MagneticAdhesionSystem")
