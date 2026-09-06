#pragma once
#include "fragment_census_reference.hpp"
#include <glm/glm.hpp>

struct FragmentPlanReference
{
  std::vector<FragmentComponent> components;
  std::map<daxa_u32,daxa_u32> remap;
};
inline FragmentPlanReference fragment_plan_reference(std::span<FragmentComponent const> raw)
{
  std::vector<FragmentComponent> sources(raw.begin(),raw.end()), targets;
  std::sort(sources.begin(),sources.end(),[](auto const &a,auto const &b){return a.label<b.label;});
  for (auto const &s:sources) if (s.count>=3u) targets.push_back(s);
  if (targets.empty() && !sources.empty())
    targets.push_back(*std::max_element(sources.begin(),sources.end(),
        [](auto const &a,auto const &b){return a.count<b.count;}));
  FragmentPlanReference out{targets,{}};
  auto center=[](FragmentComponent const &s) {
    return glm::vec3(float(double(s.sum_x)/s.count),float(double(s.sum_y)/s.count),float(double(s.sum_z)/s.count));
  };
  for (auto const &s:sources)
  {
    if (s.count>=3u) { out.remap[s.label]=s.label;continue; }
    auto cc=center(s);size_t best=0;double distance=1e30;
    for (size_t i=0;i<targets.size();++i)
    {
      auto d=cc-center(targets[i]);double next=glm::dot(d,d);
      if (next<distance) { distance=next;best=i; }
    }
    auto &dest=out.components[best];out.remap[s.label]=dest.label;
    if (s.label==dest.label) continue;
    dest.count+=s.count;dest.sum_x+=s.sum_x;dest.sum_y+=s.sum_y;dest.sum_z+=s.sum_z;
    dest.lo_x=std::min(dest.lo_x,s.lo_x);dest.lo_y=std::min(dest.lo_y,s.lo_y);dest.lo_z=std::min(dest.lo_z,s.lo_z);
    dest.hi_x=std::max(dest.hi_x,s.hi_x);dest.hi_y=std::max(dest.hi_y,s.hi_y);dest.hi_z=std::max(dest.hi_z,s.hi_z);
  }
  std::sort(out.components.begin(),out.components.end(),[](auto const &a,auto const &b){
    return a.count!=b.count ? a.count>b.count : a.label<b.label;
  });
  return out;
}
