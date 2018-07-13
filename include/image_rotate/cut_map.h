#ifndef CUT_MAP_H
#define CUT_MAP_H
#include "submap.h"
#include "cost_values.h"
class CutMap
{
	using Map = nav_msgs::OccupancyGrid;
public:
	void operator()(const Map & m, Map & result)
	{
		uint32_t i = 0, j = 0, l, r, t, d;
		uint32_t w = m.info.width;
		uint32_t h = m.info.height;
		while (j < h && IsRowNotExplored(m, j)) ++j;
		t = 0 < j ? (j - 1) : 0;
		while (j < h && !IsRowNotExplored(m, j)) ++j;
		d = j < h ? j : (h - 1);
		while (i < w && IsColumnNotExplored(m, t, d, i)) i++;
		l = 0 < i ? (i - 1) : 0;
		while (i < w && !IsColumnNotExplored(m, t, d, i)) i++;
		r = i < w ? i : (w - 1);
		submap::Submap get_submap;
		get_submap(m, l, r, t, d, result);
	}
private:
	bool IsRowNotExplored(const Map& m, uint32_t index)
	{
		auto start_iter = m.data.begin() + index * m.info.width;
		auto end_iter = start_iter + m.info.width;
		for (auto iter = start_iter; iter < end_iter; iter++)
		{
            if (costmap_2d::NO_INFORMATION != (uint8_t)(*iter))
				return false;
		}
		return true;
	}
	bool IsColumnNotExplored(const Map & m, uint32_t t, uint32_t d, uint32_t index)
	{
		auto start_iter = m.data.begin() + t * m.info.width + index;
		auto end_iter = m.data.begin() + (d + 1) * m.info.width + index;
		for (auto iter = start_iter; iter < end_iter; iter += m.info.width)
		{
            if (costmap_2d::NO_INFORMATION != (uint8_t)(*iter))
				return false;
		}
		return true;
	}

};

#endif // !CUT_MAP_H
