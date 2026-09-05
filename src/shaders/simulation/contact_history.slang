#pragma once
#include "shared.inl"

extension Manifold {
  [mutating] void compute_basis()
  {
    if (abs(normal.x) > abs(normal.y)) {
      tangent_vectors[0] = daxa_f32vec3(normal.z, 0, -normal.x);
    } else {
      tangent_vectors[0] = daxa_f32vec3(0, -normal.z, normal.y);
    }

    tangent_vectors[0] = normalize(tangent_vectors[0]);
    tangent_vectors[1] = cross(tangent_vectors[0], normal);
  }

static const daxa_f32 POSITION_TOLERANCE = 0.1f;

  bool is_same_contact(Contact* old_contact, FeaturePair contact_fp, daxa_f32vec3 new_position)
{
    bool features_match = (old_contact->fp.in_incident == contact_fp.in_incident) &&
                          (old_contact->fp.out_incident == contact_fp.out_incident);
    // POSITION GATE (was commented out): the fp compares only the INCIDENT ids, and two
    // different contacts of the same clip can share them - a feature MIS-PAIR consumes
    // the wrong old contact (yanking identity across the manifold) and strands the true
    // successor, which re-enters fresh (measured at rest: fa=[f1-f2 n4 o4] churn that
    // survived swapped-order matching AND min-distance proximity - only a wrong
    // CONSUMPTION explains a leftover with no unconsumed neighbor). A true pair sits
    // sub-mm apart at rest; 0.1 even tolerates a 2 m/s slide (33mm/frame). Mis-pairs
    // fall through to the min-distance pass, which pairs by position correctly.
    daxa_f32vec3 dpos = old_contact->position - new_position;
    bool positions_close = dot(dpos, dpos) < POSITION_TOLERANCE * POSITION_TOLERANCE;

    return features_match && positions_close;
}

  bool is_valid_contact(FeaturePair a) {
    return a.in_reference != -1 && a.out_reference != -1 && a.in_incident != -1 && a.out_incident != -1;
  }

  void key_to_fp(daxa_u32 key, out FeaturePair fp) {
    fp.in_reference = key;
    fp.out_reference = key;
    fp.in_incident = key;
    fp.out_incident = key;
  }

  [mutating]
  void update_from_old(Manifold* old_manifold, daxa_u32 swapped = 0u) {
    // With a different key the normal/feature interpretation changed - and the contact
    // feature ids are not comparable across axes, so cross-key fp matches would be
    // false positives. But REJECTING the whole manifold meant tilted deep stacks (the
    // box pool), whose SAT winner flaps between near-tied axes, lost ALL their lambdas
    // on every flap (measured MISS storms re-waking the pile). For OBB manifolds the
    // physical contact is the same while the planes agree: accept the cross-key carry,
    // skip the fp pass (ids incomparable) and let PROXIMITY inheritance move the
    // lambdas. Voxel manifolds (90-95) are per-axis by design and stay key-exact.
    //
    // SWAPPED roles (the old manifold stored the pair as (b,a) after a sort-order flip):
    // physically the same contact seen from the other side - the old normal points the
    // opposite way and the anchors belong to the opposite bodies. The hand-off below:
    // dot-gate against the FLIPPED old normal, fp pass skipped (incident ids live on the
    // other body now), anchors swapped at inheritance, and the WORLD tangent impulse
    // negated before projecting into the new basis. Normal lambda remains nonnegative.
    daxa_f32 old_n_sign = swapped != 0u ? -1.0f : 1.0f;
    bool cross_key = key != old_manifold->key || swapped != 0u;
    if (cross_key) {
      bool any_voxel = (key >= 90 && key <= 95) || (old_manifold->key >= 90 && old_manifold->key <= 95);
      if (any_voxel) { return; }
      if (dot(normal, old_manifold->normal * old_n_sign) <= 0.7f) { return; } // plane really changed
    }
    // Keep the newly measured voxel normal: its depths were computed/projected along
    // that direction. Freezing a world normal until a 25-degree threshold makes rotating
    // contacts solve the wrong plane, then switch abruptly. Reject incompatible history.
    if (key >= 90 && key <= 95 && dot(normal, old_manifold->normal) <= 0.7f) return;
    daxa_f32vec3 otv0  = old_manifold->tangent_vectors[0];
    daxa_f32vec3 otv1  = old_manifold->tangent_vectors[1];

    bool voxel_m = key >= 90 && key <= 95;
    daxa_u32 used_old = 0u;    // old contacts already inherited (never duplicate lambdas)
    daxa_u32 matched_new = 0u; // new contacts already matched by feature ids
    for(daxa_u32 i = 0u; cross_key == false && i < contact_count; i++) {
      for(daxa_u32 j = 0u; j < old_manifold->contact_count; j++) {
        if ((used_old & (1u << j)) != 0u) { continue; }
        Contact* old_contact = &old_manifold->contacts[j];
        if(is_same_contact(old_contact, contacts[i].fp, contacts[i].position)) {
            contacts[i].normal_impulse = old_contact->normal_impulse * (voxel_m ? max(0.0f, dot(normal, old_manifold->normal)) : 1.0f);
            daxa_f32vec2 ti = reproject_tangent_impulse(otv0, otv1,
                daxa_f32vec2(old_contact->tangent_impulse[0], old_contact->tangent_impulse[1]),
                tangent_vectors[0], tangent_vectors[1], old_n_sign);
            contacts[i].tangent_impulse[0] = ti.x;
            contacts[i].tangent_impulse[1] = ti.y;
            // AVBD: the penalty parameters must persist across steps (PGS overwrites these
            // fields in pre_step, so copying them is harmless for the PGS family)
            contacts[i].mass_coefficient = old_contact->mass_coefficient;
            contacts[i].tangent_mass[0] = old_contact->tangent_mass[0];
            contacts[i].tangent_mass[1] = old_contact->tangent_mass[1];
            // AVBD sticking (reference initialize()): if the contact was sticking last step,
            // keep its anchored contact points for true static friction
            contacts[i].stick = old_contact->stick;
            if (old_contact->stick != 0u)
            {
              contacts[i].anchor_a = old_contact->anchor_a;
              contacts[i].anchor_b = old_contact->anchor_b;
            }
            // Keep the legacy OBB position hysteresis. Voxel depths and normals refer
            // to the freshly measured point, so do not freeze it in world space.
            // Sticking anchors above remain body-local and persist independently.
            {
              daxa_f32vec3 dpos = contacts[i].position - old_contact->position;
              if (!voxel_m && dot(dpos, dpos) < 0.01f) // OBB legacy hysteresis only
              {
                contacts[i].position = old_contact->position;
              }
            }

            contacts[i].warm_start = min(255, old_contact->warm_start + 1u);
            used_old |= 1u << j;
            matched_new |= 1u << i;
          break;
        }
      }
    }
    // PROXIMITY inheritance for the leftovers (persistent-manifold style), ALL manifold
    // kinds. Voxel: the per-region winner voxel can hop to a neighbor on micro-motion.
    // OBB SAT: aligned deep stacks (the box pool) churn clip-vertex feature ids on
    // micro-sliding - measured 380-530 lambda resets per step, which starves deep
    // columns of converged contact forces. If an unmatched NEW contact lies near an
    // unconsumed OLD one, it inherits the old contact's physics AND identity, so the
    // solver sees one continuous contact regardless of feature-id churn.
    //
    // GLOBAL min-distance-first assignment (not greedy by contact index): SLIVER clips
    // (tilted edge-on-face pockets) emit 4-5 near-collinear contacts spaced closer than
    // the radius, where index-order greedy lets the wrong new contact consume an old one
    // and strands the true successor (measured: rest churn fa=[f1-f2 n4 o4], the blue
    // never-converging points at the pile bottom). Min-first is stable whenever frame
    // motion << contact spacing: every true pair sits at ~0mm and wins before any
    // cross-pairing at >= one spacing can fire.
    {
      daxa_f32 radius2 = voxel_m ? 0.36f // (0.6)^2: one FULL scene voxel - winner hops
                                         // move a contact a whole voxel (0.5)
                                 : 0.1225f; // (0.35)^2: SAT clip corners sit ~0.8-1.0
                                            // apart on unit cubes
      for (daxa_u32 round = 0u; round < daxa_u32(max(contact_count, 0)); ++round) {
        daxa_f32 best_d2 = radius2;
        daxa_i32 best_i = -1;
        daxa_i32 best_j = -1;
        for(daxa_u32 i = 0u; i < contact_count; i++) {
          if ((matched_new & (1u << i)) != 0u) { continue; }
          for(daxa_u32 j = 0u; j < old_manifold->contact_count; j++) {
            if ((used_old & (1u << j)) != 0u) { continue; }
            daxa_f32vec3 dpos = contacts[i].position - old_manifold->contacts[j].position;
            daxa_f32 d2 = dot(dpos, dpos);
            if (d2 < best_d2) { best_d2 = d2; best_i = daxa_i32(i); best_j = daxa_i32(j); }
          }
        }
        if (best_i < 0) { break; }
        {
          daxa_u32 i = daxa_u32(best_i);
          Contact* old_contact = &old_manifold->contacts[daxa_u32(best_j)];
          contacts[i].normal_impulse = old_contact->normal_impulse * (voxel_m ? max(0.0f, dot(normal, old_manifold->normal)) : 1.0f);
          daxa_f32vec2 ti = reproject_tangent_impulse(otv0, otv1,
              daxa_f32vec2(old_contact->tangent_impulse[0], old_contact->tangent_impulse[1]),
              tangent_vectors[0], tangent_vectors[1], old_n_sign);
          contacts[i].tangent_impulse[0] = ti.x;
          contacts[i].tangent_impulse[1] = ti.y;
          contacts[i].mass_coefficient = old_contact->mass_coefficient;
          contacts[i].tangent_mass[0] = old_contact->tangent_mass[0];
          contacts[i].tangent_mass[1] = old_contact->tangent_mass[1];
          contacts[i].stick = old_contact->stick;
          if (old_contact->stick != 0u)
          {
            // swapped roles: the old anchor_a lives in old-A's local frame = our body B
            contacts[i].anchor_a = swapped != 0u ? old_contact->anchor_b : old_contact->anchor_a;
            contacts[i].anchor_b = swapped != 0u ? old_contact->anchor_a : old_contact->anchor_b;
          }
          contacts[i].fp = old_contact->fp; // adopt the IDENTITY: matching stays stable
          contacts[i].warm_start = min(255, old_contact->warm_start + 1u);
          matched_new |= 1u << i;
          used_old |= 1u << daxa_u32(best_j);
        }
      }
    }

    // EXTRACTION TREND gauge (repurposes the never-used Manifold.error field): per-frame
    // change of the manifold's deepest penetration, in tenth-millimeters (positive =
    // extracting). The sleep depth veto releases when extraction has STALLED: a fixed
    // depth threshold cannot work alone because the achievable equilibrium varies
    // 49-70mm per landing - a pair resting 4mm past the threshold was measured held
    // awake FOREVER (430/432 plateau, maxv=0, pen 53-55 for 250+ samples).
    {
      daxa_f32 deep_new = 0.0f;
      for (daxa_u32 i = 0u; i < daxa_u32(max(contact_count, 0)); i++) {
        deep_new = min(deep_new, contacts[i].penetration);
      }
      daxa_f32 deep_old = 0.0f;
      for (daxa_u32 j = 0u; j < daxa_u32(max(old_manifold->contact_count, 0)); j++) {
        deep_old = min(deep_old, old_manifold->contacts[j].penetration);
      }
      error = daxa_i32(clamp((deep_new - deep_old) * 10000.0f, -32000.0f, 32000.0f));
    }
  }
};


func find_old_collision(daxa_u32 body_a_index, daxa_u32 body_b_index, daxa_u32 old_collision_count, Manifold* old_collisions, out daxa_u32 old_index) -> daxa_b32 {
  for(daxa_u32 i = 0u; i < old_collision_count; ++i) {
    if(old_collisions[i].obb1_index == body_a_index && old_collisions[i].obb2_index == body_b_index) {
      old_index = i;
      return true;
    }
  }
  return false;
}
