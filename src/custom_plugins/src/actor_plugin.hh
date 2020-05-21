

#ifndef SERVICESIM_PLUGINS_WANDERINGACTORPLUGIN_HH_
#define SERVICESIM_PLUGINS_WANDERINGACTORPLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include "gazebo/util/system.hh"

namespace std{
	const double EPS = 1E-9;
	struct pt {
		double x, y;
		
		pt(double _x, double _y){
			x = _x;
			y  =_y;
		}
		bool operator<(const pt& p) const
		{
			return x < p.x - EPS || (abs(x - p.x) < EPS && y < p.y - EPS);
		}
	};

	struct line {
		double a, b, c;

		line() {}
		line(pt p, pt q)
		{
			a = p.y - q.y;
			b = q.x - p.x;
			c = -a * p.x - b * p.y;
			norm();
		}

		void norm()
		{
			double z = sqrt(a * a + b * b);
			if (abs(z) > EPS)
				a /= z, b /= z, c /= z;
		}

		double dist(pt p) const { return a * p.x + b * p.y + c; }
	};

	double det(double a, double b, double c, double d)
	{
		return a * d - b * c;
	}

	inline bool betw(double l, double r, double x)
	{
		return min(l, r) <= x + EPS && x <= max(l, r) + EPS;
	}

	inline bool intersect_1d(double a, double b, double c, double d)
	{
		if (a > b)
			swap(a, b);
		if (c > d)
			swap(c, d);
		return max(a, c) <= min(b, d) + EPS;
	}

	bool intersect(ignition::math::Line2d line1, ignition::math::Line2d line2, ignition::math::Vector2d &int1)
	{	
		pt a = pt(line1[0].X(), line1[0].Y());
		pt b = pt(line1[1].X(), line1[1].Y());
		
		pt c = pt(line2[0].X(), line2[0].Y());
		pt d = pt(line2[1].X(), line2[1].Y());

		if (!intersect_1d(a.x, b.x, c.x, d.x) || !intersect_1d(a.y, b.y, c.y, d.y))
			return false;
		line m(a, b);
		line n(c, d);
		double zn = det(m.a, m.b, n.a, n.b);
		if (abs(zn) < EPS) {
			if (abs(m.dist(c)) > EPS || abs(n.dist(a)) > EPS)
				return false;
			if (b < a)
				swap(a, b);
			if (d < c)
				swap(c, d);
			pt left = max(a, c);
			pt right = min(b, d);
	  
			int1.X() = left.x;
			int1.Y() = left.y;

			return true;
		} else {
			pt left = pt(-det(m.c, m.b, n.c, n.b) / zn, -det(m.a, m.c, n.a, n.c) / zn);
			pt right = pt(-det(m.c, m.b, n.c, n.b) / zn, -det(m.a, m.c, n.a, n.c) / zn);

			int1.X() = left.x;
			int1.Y() = left.y;

			return betw(a.x, b.x, left.x) && betw(a.y, b.y, left.y) && betw(c.x, d.x, left.x) && betw(c.y, d.y, left.y);
		}
	}
}

namespace servicesim
{
  class ActorPluginPrivate;

  class GAZEBO_VISIBLE ActorPlugin : public gazebo::ModelPlugin
  {
    /// \brief Constructor
    public: ActorPlugin();

    /// \brief Load the actor plugin.
    /// \param[in] _model Pointer to the parent model.
    /// \param[in] _sdf Pointer to the plugin's SDF elements.
    public: virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
        override;

    /// \brief Function that is called every update cycle.
    /// \param[in] _info Timing information
    private: void OnUpdate(const gazebo::common::UpdateInfo &_info);

    private: ignition::math::Vector3d HandleObstacles(ignition::math::Vector3d &_pos, const gazebo::common::UpdateInfo &_info);


    /// \internal
    private: ActorPluginPrivate *dataPtr;
    
    private: ignition::math::Pose3d RandomTarget(const gazebo::common::UpdateInfo &_info);
    
  };
}
#endif
