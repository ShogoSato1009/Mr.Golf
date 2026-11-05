#include "physics/BallPhysics.hpp"
#include "physics/Collision.hpp"

#include <algorithm>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace golf;

namespace {

void check(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}

void near(double actual, double expected, double tolerance, const char* message) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance)
        throw std::runtime_error(std::string(message) + ": actual=" + std::to_string(actual) +
                                 ", expected=" + std::to_string(expected));
}

void near(Vec3 actual, Vec3 expected, double tolerance, const char* message) {
    near(length(actual - expected), 0.0, tolerance, message);
}

template<class Function>
void throws(Function f) {
    bool caught = false;
    try { f(); } catch (const std::exception&) { caught = true; }
    check(caught, "Expected an exception");
}

Environment vacuum() { Environment e; e.airDensity = 0.0; return e; }
Environment inertial() { auto e = vacuum(); e.gravity = {}; return e; }

SurfaceMaterial frictionless(double restitution = 1.0) {
    return {restitution, 0.0, 0.0, 0.0, 0.0};
}

SurfaceMaterial noRollingResistance() { return {0.5, 0.8, 0.5, 0.0, 0.0}; }

BallState rollingBall(double speed, Vec3 normal = {0.0, 1.0, 0.0}) {
    BallState s;
    s.position.y = BallParameters{}.radius / normal.y;
    s.velocity = speed * tangent({1.0, 0.0, 0.0}, normal) /
                 length(tangent({1.0, 0.0, 0.0}, normal));
    s.angularVelocity = cross(normal, s.velocity) / BallParameters{}.radius;
    return s;
}

void advance(BallState& s, const BallPhysics& p, const Terrain& t, double seconds,
             Environment e = vacuum()) {
    const auto count = static_cast<std::size_t>(std::llround(seconds / p.settings().fixedTimeStep));
    for (std::size_t i = 0; i < count; ++i) p.step(s, t, e);
}

double energy(const BallState& s, BallParameters b = {}) {
    return 0.5*b.mass*dot(s.velocity, s.velocity) +
           0.5*b.inertia()*dot(s.angularVelocity, s.angularVelocity);
}

bool sameState(const BallState& a, const BallState& b) {
    return a.position.x == b.position.x && a.position.y == b.position.y && a.position.z == b.position.z &&
           a.velocity.x == b.velocity.x && a.velocity.y == b.velocity.y && a.velocity.z == b.velocity.z &&
           a.angularVelocity.x == b.angularVelocity.x && a.angularVelocity.y == b.angularVelocity.y &&
           a.angularVelocity.z == b.angularVelocity.z && a.phase == b.phase;
}

} // namespace

int main() {
    const std::vector<std::pair<const char*, std::function<void()>>> tests = {
        {"vector operations and right-handed axes", [] {
            near(cross({1, 0, 0}, {0, 1, 0}), {0, 0, 1}, 0.0, "cross product");
            near(dot({1, 2, 3}, {4, 5, 6}), 32.0, 0.0, "dot product");
            near(tangent({1, 2, 3}, {0, 1, 0}), {1, 0, 3}, 0.0, "tangent");
        }},
        {"linear and angular impulse conversion is additive", [] {
            const BallParameters b;
            BallPhysics p;
            BallState s;
            s.velocity = {1, 2, 3};
            s.angularVelocity = {4, 5, 6};
            const Vec3 position = s.position;
            p.applyShot(s, {{2*b.mass, -b.mass, 3*b.mass},
                            {b.inertia(), 2*b.inertia(), -3*b.inertia()}});
            near(s.velocity, {3, 1, 6}, 1e-12, "J/m");
            near(s.angularVelocity, {5, 7, 3}, 1e-12, "L/I");
            near(s.position, position, 0.0, "impulse preserves position");
        }},
        {"vacuum ballistic motion matches the analytic solution", [] {
            BallPhysics p;
            BallState s;
            s.position = {1, 100, -3};
            s.velocity = {15, 20, -4};
            advance(s, p, Terrain::plane(), 2.0);
            near(s.position, {31, 100+40-0.5*9.80665*4, -11}, 1e-9, "ballistic position");
            near(s.velocity, {15, 20-9.80665*2, -4}, 1e-10, "ballistic velocity");
        }},
        {"quadratic drag matches its closed-form solution", [] {
            BallPhysics p;
            BallState s;
            s.position = {0, 100, 0};
            s.velocity = {30, 0, 0};
            Environment e;
            e.gravity = {};
            const BallParameters b;
            const double k = 0.5*e.airDensity*pi*b.radius*b.radius*0.25/b.mass;
            advance(s, p, Terrain::plane(), 2.0, e);
            near(s.velocity.x, 30/(1+k*30*2), 1e-9, "drag velocity");
            near(s.position.x, std::log1p(k*30*2)/k, 1e-9, "drag position");
            check(s.velocity.x > 0, "drag must not reverse the ball");
        }},
        {"spin decay is exponential and vanishes in vacuum", [] {
            BallPhysics p;
            BallState s;
            s.position.y = 100;
            s.angularVelocity = {10, 20, 30};
            BallState v = s;
            Environment e;
            e.gravity = {};
            advance(s, p, Terrain::plane(), 3, e);
            near(s.angularVelocity, std::exp(-0.04*1.225*3)*Vec3{10, 20, 30}, 1e-9, "spin decay");
            advance(v, p, Terrain::plane(), 3, inertial());
            near(v.angularVelocity, {10, 20, 30}, 0, "vacuum spin");
        }},
        {"wind uses relative rather than ground speed", [] {
            BallPhysics p;
            BallState a;
            a.position.y = 100;
            a.velocity = {30, 5, 3};
            a.angularVelocity = {0, 40, 200};
            BallState b = a;
            const Vec3 wind{7, 0, -4};
            b.velocity += wind;
            Environment e;
            Environment windy = e;
            windy.windVelocity = wind;
            advance(a, p, Terrain::plane(), 2, e);
            advance(b, p, Terrain::plane(), 2, windy);
            near(b.velocity, a.velocity+wind, 1e-9, "Galilean velocity");
            near(b.position, a.position+2*wind, 1e-9, "Galilean position");
            BallState carried;
            carried.position.y = 100;
            carried.velocity = wind;
            windy.gravity = {};
            advance(carried, p, Terrain::plane(), 1, windy);
            near(carried.velocity, wind, 0, "zero relative airflow");
        }},
        {"crosswind accelerates a stationary airborne ball", [] {
            BallState s;
            s.position.y = 100;
            Environment e;
            e.gravity = {};
            e.windVelocity = {0, 0, 15};
            advance(s, BallPhysics{}, Terrain::plane(), 1, e);
            check(s.velocity.z > 0 && s.velocity.z < 15, "wind direction and drag stability");
        }},
        {"backspin and sidespin have correct lift directions", [] {
            BallPhysics p;
            BallState base;
            base.position.y = 100;
            base.velocity.x = 50;
            auto top = base;
            auto back = base;
            auto side = base;
            auto axial = base;
            top.angularVelocity.z = -300;
            back.angularVelocity.z = 300;
            side.angularVelocity.y = 300;
            axial.angularVelocity.x = 300;
            Environment e;
            e.gravity = {};
            const auto terrain = Terrain::plane();
            advance(base, p, terrain, 0.5, e);
            advance(top, p, terrain, 0.5, e);
            advance(back, p, terrain, 0.5, e);
            advance(side, p, terrain, 0.5, e);
            advance(axial, p, terrain, 0.5, e);
            check(back.velocity.y > 0 && top.velocity.y < 0, "lift sign");
            check(side.velocity.z < 0, "sidespin sign");
            near(top.velocity.y, -back.velocity.y, 1e-10, "spin symmetry");
            near(axial.velocity, base.velocity, 1e-10, "axial spin must not create lift");
        }},
        {"lift alone does no translational work", [] {
            AerodynamicParameters aero;
            aero.dragCoefficient = 0;
            aero.spinDecayPerDensity = 0;
            BallPhysics p({}, aero);
            BallState s;
            s.position.y = 100;
            s.velocity.x = 50;
            s.angularVelocity.z = 600;
            Environment e;
            e.gravity = {};
            advance(s, p, Terrain::plane(), 2, e);
            near(length(s.velocity), 50, 1e-8, "lift is perpendicular to relative velocity");
        }},
        {"continuous collision preserves remaining tick time", [] {
            PhysicsSettings settings;
            settings.fixedTimeStep = 0.02;
            BallPhysics p({}, {}, settings);
            BallState s;
            s.position.y = 1 + p.ball().radius;
            s.velocity.y = -100;
            p.step(s, Terrain::plane(0, 0, 0, frictionless()), inertial());
            near(s.velocity.y, 100, 1e-10, "elastic bounce");
            near(s.position.y, 1+p.ball().radius, 1e-8, "remaining flight time");
            check(s.phase == MotionPhase::Airborne, "bounce phase");
        }},
        {"restitution dissipates normal kinetic energy", [] {
            BallState s;
            s.velocity = {0, -10, 0};
            const double before = energy(s);
            collision::resolveImpact(s, {0, 1, 0}, {}, frictionless(0.4), 0.2);
            near(s.velocity.y, 4, 1e-12, "normal restitution");
            near(energy(s)/before, 0.16, 1e-12, "normal energy ratio");
        }},
        {"oblique impact transfers linear momentum into spin", [] {
            BallState s;
            s.velocity = {10, -2, 0};
            SurfaceMaterial m{0.5, 0.3, 0.2, 0, 0};
            collision::resolveImpact(s, {0, 1, 0}, {}, m, 0.2);
            near(s.velocity, {9.4, 1, 0}, 1e-12, "Coulomb-limited impulse");
            near(s.angularVelocity.z, -0.6/(0.4*BallParameters{}.radius), 1e-10, "impact torque");
        }},
        {"sticking impact removes contact slip without reversing it", [] {
            BallState s;
            s.velocity = {1, -5, 0};
            collision::resolveImpact(s, {0, 1, 0}, {}, noRollingResistance(), 0.2);
            near(s.velocity.x, 5.0/7.0, 1e-12, "effective tangential mass");
            near(collision::slipVelocity(s, {0, 1, 0}, BallParameters{}.radius), {}, 1e-12, "sticking");
        }},
        {"impact friction never adds total kinetic energy", [] {
            auto turf = materialFor(SurfaceType::Fairway);
            turf.impactBraking = 0.55;
            for (double spin : {-800.0, -50.0, 0.0, 300.0, 1000.0}) {
                for (const auto& material : {materialFor(SurfaceType::Fairway), turf}) {
                    BallState s;
                    s.velocity = {10, -5, -2};
                    s.angularVelocity = {spin/3, spin/2, spin};
                    const double before = energy(s);
                    collision::resolveImpact(s, {0, 1, 0}, {}, material, 0.2);
                    check(energy(s) <= before+1e-12, "impact created energy");
                }
            }
        }},
        {"turf impact braking is capped at tangential speed", [] {
            auto m = frictionless(0.22);
            m.impactBraking = 0.55;
            BallState s;
            s.velocity = {2, -10, 0};
            collision::resolveImpact(s, {0, 1, 0}, {}, m, 0.2);
            near(s.velocity, {0, 2.2, 0}, 1e-12, "braking cannot reverse travel");
        }},
        {"pure rolling conserves speed without resistance", [] {
            BallState s = rollingBall(2);
            BallPhysics p;
            advance(s, p, Terrain::plane(0, 0, 0, noRollingResistance()), 2);
            near(s.position.x, 4, 1e-9, "rolling distance");
            near(s.velocity.x, 2, 1e-10, "rolling speed");
            near(collision::slipVelocity(s, {0, 1, 0}, p.ball().radius), {}, 1e-10, "no slip");
            check(s.phase == MotionPhase::Rolling, "rolling phase");
        }},
        {"sliding converges to analytic solid-sphere rolling speed", [] {
            BallState s;
            s.velocity.x = 3;
            BallPhysics p;
            const double before = energy(s);
            advance(s, p, Terrain::plane(0, 0, 0, noRollingResistance()), 1);
            near(s.velocity.x, 3*5.0/7.0, 1e-10, "sliding to rolling speed");
            check(s.phase == MotionPhase::Rolling, "sliding transitioned to rolling");
            check(energy(s) < before, "sliding must dissipate energy");
        }},
        {"spin at rest can propel the ball through ground friction", [] {
            BallState s;
            s.angularVelocity.z = -100;
            advance(s, BallPhysics{}, Terrain::plane(0, 0, 0, noRollingResistance()), 1);
            near(s.velocity.x, (2.0/7.0)*100*BallParameters{}.radius, 1e-9, "spin to translation");
            check(s.position.x > 0, "spin must move the centre");
        }},
        {"rolling resistance stops without reversing direction", [] {
            SurfaceMaterial m = noRollingResistance();
            m.rollingResistance = 0.05;
            const auto terrain = Terrain::plane(0, 0, 0, m);
            BallPhysics p;
            BallState s = rollingBall(1);
            double previousEnergy = energy(s);
            for (int i = 0; i < 1440; ++i) {
                p.step(s, terrain, vacuum());
                check(s.velocity.x >= -1e-12, "rolling resistance reversed velocity");
                check(energy(s) <= previousEnergy+1e-12, "ground resistance created energy");
                previousEnergy = energy(s);
            }
            const double deceleration = m.rollingResistance*9.80665/1.4;
            near(s.position.x, 1/(2*deceleration), 0.005, "rolling stopping distance");
            near(s.velocity, {}, 0, "stopped translation");
            near(s.angularVelocity, {}, 0, "stopped spin");
            check(s.phase == MotionPhase::Resting, "resting phase");
        }},
        {"rolling torque cannot slow translation on frictionless ground", [] {
            auto m = frictionless();
            m.rollingResistance = 0.1;
            BallState s = rollingBall(1);
            advance(s, BallPhysics{}, Terrain::plane(0, 0, 0, m), 1);
            near(s.velocity.x, 1, 1e-12, "no invented tangential force");
            near(s.angularVelocity, {}, 1e-10, "rolling torque damps spin");
            check(s.phase == MotionPhase::Sliding, "frictionless motion is now sliding");
        }},
        {"normal-axis spin decays and stops", [] {
            BallState s;
            s.angularVelocity.y = 30;
            advance(s, BallPhysics{}, Terrain::plane(), 5);
            near(s.angularVelocity, {}, 0, "torsional friction");
            near(s.position.x, 0, 0, "normal spin does not translate");
            check(s.phase == MotionPhase::Resting, "spinning ball comes to rest");
        }},
        {"rolling slope acceleration is g*sin(theta)/(1+k)", [] {
            const auto terrain = Terrain::plane(0, -0.1, 0, noRollingResistance());
            const Vec3 n = terrain.sample(0, 0).normal;
            BallState s = rollingBall(0, n);
            const Vec3 initial = s.position;
            const Vec3 a = tangent(vacuum().gravity, n)/1.4;
            advance(s, BallPhysics{}, terrain, 1);
            near(s.velocity, a, 1e-9, "slope acceleration");
            near(s.position, initial+0.5*a, 1e-8, "slope position");
            check(s.phase == MotionPhase::Rolling, "slope rolling");
        }},
        {"gentle slopes hold while steep slopes restart rolling", [] {
            auto m = noRollingResistance();
            m.rollingResistance = 0.03;
            const auto gentle = Terrain::plane(0, -0.01, 0, m);
            BallState s = rollingBall(0, gentle.sample(0, 0).normal);
            advance(s, BallPhysics{}, gentle, 2);
            near(s.position.x, 0, 1e-12, "gentle slope holding");
            check(s.phase == MotionPhase::Resting, "gentle slope resting");
            const auto steep = Terrain::plane(0, -0.2, 0, m);
            s.position.y = BallParameters{}.radius/steep.sample(0, 0).normal.y;
            advance(s, BallPhysics{}, steep, 1);
            check(s.velocity.x > 0.5, "changed slope must wake the ball");
        }},
        {"uphill rolling reverses naturally on a slope", [] {
            const auto terrain = Terrain::plane(0, 0.1, 0, noRollingResistance());
            BallState s = rollingBall(1, terrain.sample(0, 0).normal);
            advance(s, BallPhysics{}, terrain, 3);
            check(s.velocity.x < -1, "gravity must restart downhill motion");
        }},
        {"surface presets order stopping distances", [] {
            std::vector<double> distances;
            for (auto type : {SurfaceType::Green, SurfaceType::Fairway, SurfaceType::Rough, SurfaceType::Sand}) {
                BallState s = rollingBall(1);
                advance(s, BallPhysics{}, Terrain::plane(0, 0, 0, materialFor(type)), 20);
                check(s.phase == MotionPhase::Resting, "all surfaces must stop");
                distances.push_back(s.position.x);
            }
            check(distances[0] > distances[1] && distances[1] > distances[2] && distances[2] > distances[3],
                  "green > fairway > rough > sand stopping distance");
        }},
        {"material is queried again when crossing a boundary", [] {
            Terrain mixed([](double x, double) {
                return TerrainSample{0, {0, 1, 0}, materialFor(x < 0.3 ? SurfaceType::Green : SurfaceType::Sand)};
            });
            BallState a = rollingBall(1);
            BallState b = a;
            advance(a, BallPhysics{}, mixed, 20);
            advance(b, BallPhysics{}, Terrain::plane(0, 0, 0, materialFor(SurfaceType::Green)), 20);
            check(a.position.x > 0.3 && a.position.x < b.position.x*0.5, "material boundary response");
        }},
        {"smooth curved terrain stays in contact", [] {
            Terrain terrain([](double x, double z) {
                return TerrainSample{0.03*std::sin(x)+0.01*std::sin(z),
                                     {-0.03*std::cos(x), 1, -0.01*std::cos(z)}, noRollingResistance()};
            });
            BallState s = rollingBall(1, terrain.sample(0, 0).normal);
            BallPhysics p;
            for (int i = 0; i < 1200; ++i) {
                p.step(s, terrain, vacuum());
                const auto surface = terrain.sample(s.position.x, s.position.z);
                near((s.position.y-surface.height)*surface.normal.y, p.ball().radius, 1e-9, "surface gap");
                near(dot(s.velocity, surface.normal), 0, 1e-10, "contact normal velocity");
            }
        }},
        {"fast impacts do not tunnel through ground", [] {
            PhysicsSettings settings;
            settings.fixedTimeStep = 1.0/60;
            settings.maxSubstep = settings.fixedTimeStep;
            BallPhysics p({}, {}, settings);
            BallState s;
            s.position.y = 1+p.ball().radius;
            s.velocity = {100, -2000, 0};
            p.step(s, Terrain::plane(0, 0, 0, frictionless(0.8)), inertial());
            near(s.velocity.y, 1600, 1e-8, "high-speed restitution");
            near(s.position.y, p.ball().radius+1600*(settings.fixedTimeStep-0.0005), 1e-7, "high-speed impact time");
        }},
        {"low-speed bounce and initial overlap settle without jitter", [] {
            BallState s;
            s.position.y = -1;
            s.velocity.y = -0.01;
            BallPhysics p;
            const auto terrain = Terrain::plane();
            for (int i = 0; i < 10000; ++i) p.step(s, terrain, vacuum());
            near(s.position.y, p.ball().radius, 0, "overlap recovery");
            near(s.velocity, {}, 0, "no resting jitter");
            check(s.phase == MotionPhase::Resting, "stable rest");
        }},
        {"a changed wind wakes a resting ball", [] {
            BallState s;
            BallPhysics p;
            const auto terrain = Terrain::plane();
            advance(s, p, terrain, 1);
            check(s.phase == MotionPhase::Resting, "initial rest");
            Environment e;
            e.windVelocity.x = 60;
            advance(s, p, terrain, 1, e);
            check(s.velocity.x > 1 && s.position.x > 0, "wind wakes rest");
        }},
        {"RK4 flight converges when the time step is halved", [] {
            const auto simulate = [](double dt) {
                PhysicsSettings settings;
                settings.fixedTimeStep = dt;
                settings.maxSubstep = dt;
                settings.maxTravelPerSubstep = 100;
                BallState s;
                s.position.y = 200;
                s.velocity = {65, 15, 3};
                s.angularVelocity = {10, 80, 400};
                advance(s, BallPhysics({}, {}, settings), Terrain::plane(), 2, Environment{});
                return s;
            };
            const auto reference = simulate(1.0/1920);
            const double coarseError = length(simulate(1.0/30).position-reference.position);
            const double fineError = length(simulate(1.0/60).position-reference.position);
            check(coarseError < 1e-4 && fineError < coarseError*0.2, "fourth-order flight convergence");
        }},
        {"same fixed ticks produce bit-identical state", [] {
            BallPhysics p;
            BallState a;
            p.applyShot(a, {{2.8, 0.9, 0.1}, {0, 0.00002, 0.00015}});
            BallState b = a;
            const auto terrain = Terrain::plane();
            for (int i = 0; i < 2400; ++i) p.step(a, terrain);
            for (int frame = 0; frame < 600; ++frame)
                for (int tick = 0; tick < 4; ++tick) p.step(b, terrain);
            check(sameState(a, b), "tick grouping must not change simulation");
        }},
        {"full shot remains finite, above ground, and eventually stops", [] {
            BallPhysics p;
            BallState s;
            p.applyShot(s, {{p.ball().mass*65, p.ball().mass*20, 0},
                            {0, p.ball().inertia()*20, p.ball().inertia()*250}});
            const auto terrain = Terrain::plane();
            Environment e;
            e.windVelocity = {1, 0, 2};
            for (int i = 0; i < 14400; ++i) {
                p.step(s, terrain, e);
                check(isFinite(s.position) && isFinite(s.velocity) && isFinite(s.angularVelocity), "finite shot state");
                check(s.position.y >= p.ball().radius-1e-8, "shot penetrated ground");
            }
            check(s.position.x > 20, "shot must travel");
            check(s.phase == MotionPhase::Resting, "shot must settle");
        }},
        {"invalid parameters and terrain are rejected", [] {
            BallParameters b;
            b.mass = 0;
            throws([&] { BallPhysics p(b); });
            b = {};
            b.radius = std::numeric_limits<double>::infinity();
            throws([&] { BallPhysics p(b); });
            AerodynamicParameters a;
            a.dragCoefficient = -1;
            throws([&] { BallPhysics p({}, a); });
            PhysicsSettings settings;
            settings.fixedTimeStep = 0;
            throws([&] { BallPhysics p({}, {}, settings); });
            settings = {};
            settings.collisionIterations = 65;
            throws([&] { BallPhysics p({}, {}, settings); });
            throws([] { Terrain t(Terrain::Sampler{}); });
            throws([] { (void)Terrain::plane(0, 100); });
            auto m = noRollingResistance();
            m.dynamicFriction = 2;
            throws([&] { (void)Terrain::plane(0, 0, 0, m); });
            Terrain bad([](double, double) { return TerrainSample{0, {0, 0, 0}, {}}; });
            throws([&] { (void)bad.sample(0, 0); });
        }},
        {"failed steps and impulses leave caller state unchanged", [] {
            BallPhysics p;
            BallState s;
            s.velocity.x = 10;
            const auto before = s;
            Environment bad;
            bad.airDensity = -1;
            throws([&] { p.step(s, Terrain::plane(), bad); });
            check(sameState(s, before), "invalid environment mutated state");
            const double nan = std::numeric_limits<double>::quiet_NaN();
            throws([&] { p.applyShot(s, {{nan, 0, 0}, {}}); });
            check(sameState(s, before), "invalid shot mutated state");
            Terrain failing([](double x, double) {
                return TerrainSample{0, x > 0 ? Vec3{} : Vec3{0, 1, 0}, {}};
            });
            throws([&] { p.step(s, failing); });
            check(sameState(s, before), "mid-step failure mutated state");
            PhysicsSettings settings;
            settings.maxSubsteps = 1;
            BallPhysics limited({}, {}, settings);
            throws([&] { limited.step(s, Terrain::plane()); });
            check(sameState(s, before), "budget exhaustion mutated state");
        }},
    };

    std::size_t failures = 0;
    for (const auto& test : tests) {
        try {
            test.second();
            std::cout << "PASS " << test.first << '\n';
        } catch (const std::exception& e) {
            ++failures;
            std::cerr << "FAIL " << test.first << ": " << e.what() << '\n';
        }
    }
    std::cout << (tests.size()-failures) << '/' << tests.size() << " tests passed\n";
    return failures == 0 ? 0 : 1;
}
