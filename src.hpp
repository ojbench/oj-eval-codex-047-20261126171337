#ifndef PPCA_SRC_HPP
#define PPCA_SRC_HPP
#include "math.h"
#include <vector>
#include <algorithm>

class Controller {
public:
    Controller(const Vec &_pos_tar, double _v_max, double _r, int _id, Monitor *_monitor) {
        pos_tar = _pos_tar;
        v_max = _v_max;
        r = _r;
        id = _id;
        monitor = _monitor;
    }
    void set_pos_cur(const Vec &_pos_cur) { pos_cur = _pos_cur; }
    void set_v_cur(const Vec &_v_cur) { v_cur = _v_cur; }
private:
    int id;
    Vec pos_tar;
    Vec pos_cur;
    Vec v_cur;
    double v_max, r;
    Monitor *monitor;
    int stagnant = 0, escape_left = 0, escapes = 0;
    double best_distance = 1e100;
    Vec escape_direction;

    struct Plane { Vec n; double b; }; // Feasible velocities satisfy n dot v <= b.
    double square(const Vec &a) const { return a.dot(a); }
    Vec limit(Vec a) const {
        double length = std::sqrt(square(a));
        return length > v_max ? a * (v_max / length) : a;
    }
    bool project(const std::vector<Plane> &planes, Vec preferred, Vec &answer) const {
        answer = limit(preferred);
        for (int i = 0; i < int(planes.size()); ++i) {
            const Plane &p = planes[i];
            if (p.n.dot(answer) <= p.b + 1e-9) continue;
            double discriminant = v_max * v_max - p.b * p.b;
            if (discriminant < -1e-9) return false;
            Vec base = p.n * p.b, tangent(-p.n.y, p.n.x);
            double lo = -std::sqrt(std::max(0.0, discriminant)), hi = -lo;
            for (int j = 0; j < i; ++j) {
                double a = planes[j].n.dot(tangent);
                double b = planes[j].b - planes[j].n.dot(base);
                if (std::abs(a) < 1e-12) {
                    if (b < -1e-9) return false;
                } else if (a > 0) hi = std::min(hi, b / a);
                else lo = std::max(lo, b / a);
            }
            if (lo > hi + 1e-9) return false;
            double t = std::max(lo, std::min(hi, tangent.dot(preferred - base)));
            answer = base + tangent * t;
        }
        return true;
    }
public:
    Vec get_v_next() {
        Vec goal = pos_tar - pos_cur;
        double distance_to_goal = goal.norm();
        if (distance_to_goal < best_distance - std::max(0.002, v_max * TIME_INTERVAL * 0.05)) {
            best_distance = distance_to_goal;
            stagnant = 0;
        } else if (distance_to_goal > 0.009) ++stagnant;
        if (stagnant > 160 && escape_left == 0) {
            Vec away;
            for (int j = 0; j < monitor->get_robot_number(); ++j) {
                if (j == id) continue;
                Vec offset = pos_cur - monitor->get_pos_cur(j);
                away += offset / std::max(1e-6, square(offset));
            }
            if (square(away) < 1e-12) away = Vec(std::cos(id * 2.4), std::sin(id * 2.4));
            escape_direction = away.normalize();
            double angle = 1.5 * std::sin(escapes++ * 2.399963);
            escape_direction = Vec(escape_direction.x * std::cos(angle) - escape_direction.y * std::sin(angle),
                                   escape_direction.x * std::sin(angle) + escape_direction.y * std::cos(angle));
            escape_left = 200 + int(4 * r / std::max(0.01, v_max * TIME_INTERVAL));
            stagnant = 0;
        }
        if (escape_left > 0) {
            --escape_left;
            stagnant = 0;
            goal = escape_direction * (v_max * 10 + r * 4);
            best_distance = distance_to_goal;
        }
        Vec preferred = limit(goal / TIME_INTERVAL);
        std::vector<Plane> safe, avoidance;
        bool blocked = false;
        double nearest = 1e100;
        Vec detour;
        const double horizon = 2.0;
        for (int j = 0; j < monitor->get_robot_number(); ++j) {
            if (j == id) continue;
            Vec p = monitor->get_pos_cur(j) - pos_cur;
            double d2 = square(p), distance = std::sqrt(d2);
            double radius = r + monitor->get_r(j);
            if (distance < 1e-12) continue;
            Vec normal = p / distance;
            // Both robots reserve half the gap, making the complete step safe.
            safe.push_back({normal, std::max(0.0, distance - radius) / (2 * TIME_INTERVAL)});
            double along = p.dot(goal) / goal.norm();
            if (along > 0 && along < goal.norm() && distance < radius + v_max * horizon &&
                std::abs(p.cross(goal)) / goal.norm() < radius * 1.05 + 0.02 && distance - radius < nearest) {
                blocked = true;
                nearest = distance - radius;
                double sine = std::min(1.0, (radius * 1.05 + 0.02) / distance);
                detour = (normal * std::sqrt(1 - sine * sine) + Vec(normal.y, -normal.x) * sine) * v_max;
            }
            Vec relative = monitor->get_v_cur(id) - monitor->get_v_cur(j);
            Vec u, direction;
            if (d2 > radius * radius) {
                Vec w = relative - p / horizon;
                double w2 = square(w), dot = w.dot(p);
                if (dot < 0 && dot * dot > radius * radius * w2) {
                    double length = std::sqrt(w2);
                    Vec unit = length > 1e-12 ? w / length : -normal;
                    direction = Vec(unit.y, -unit.x);
                    u = unit * (radius / horizon - length);
                } else {
                    double leg = std::sqrt(std::max(0.0, d2 - radius * radius));
                    if (p.cross(w) > 0)
                        direction = Vec(p.x * leg - p.y * radius, p.x * radius + p.y * leg) / d2;
                    else
                        direction = -Vec(p.x * leg + p.y * radius, -p.x * radius + p.y * leg) / d2;
                    u = direction * relative.dot(direction) - relative;
                }
            } else {
                Vec w = relative - p / TIME_INTERVAL;
                double length = std::sqrt(square(w));
                Vec unit = length > 1e-12 ? w / length : -normal;
                direction = Vec(unit.y, -unit.x);
                u = unit * (radius / TIME_INTERVAL - length);
            }
            Vec n(direction.y, -direction.x);
            avoidance.push_back({n, n.dot(monitor->get_v_cur(id) + u * 0.5)});
        }
        if (blocked) preferred = detour;
        std::vector<Plane> constraints = safe;
        constraints.insert(constraints.end(), avoidance.begin(), avoidance.end());
        Vec answer;
        if (!project(constraints, preferred, answer)) project(safe, preferred, answer);
        return limit(answer);
    }
};
#endif
