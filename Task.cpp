//***************************************************************************
// Copyright 2007-2020 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Jaehyeong Hwang                                                       *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include <vector>

namespace Maneuver
{

  namespace Test
  {
    using DUNE_NAMESPACES;
    using std::vector; //vector will be used for generating way-points

    struct Arguments
    {
      uint16_t connection_timeout;
      float waiting_time;
      float h;
      float s;
      float current_lat;
      float current_lon;
    };


    struct Task: public DUNE::Tasks::Task
    {
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Arguments m_args;
      float start_time; //10sec

      IMC::Reference m_ref;
      IMC::FollowReference m_follow_ref;
      IMC::EstimatedState m_estate;
      IMC::FollowRefState m_ref_state;
      IMC::PlanControlState m_plan_control_state;

      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),

      {
        param("Waiting time", m_args.waiting_time)
        .defaultValue("10.0")
        .units(Units::Second)
        .description("Waiting time before starting Follow reference after boot");

        param("Longitudinal distance", m_args.h)
        .defaultValue("1.0")
        .units(Units::Meter)
        .description("Longitudinal distance vehicle has to go to the next waypoint");

        param("Latitudinal distance", m_args.s)
        .defaultValue("1.0")
        .units(Units::Meter)
        .description("Latitudinal distance vehicle has to go to the next waypoint");

        param("Caravela Connection Timeout", m_args.connection_timeout)
        .defaultValue("1.0")
        .units(Units::Meter)
        .description("Latitudinal distance vehicle has to go to the next waypoint");

        bind<IMC::FollowRefState>(this);
        bind<IMC::EstimatedState>(this);
        bind<IMC::Abort>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
      }

      //! Reserve entity identifiers.
      void
      onEntityReservation(void)
      {
      }

      //! Resolve entity names.
      void
      onEntityResolution(void)
      {
      }

      //! Acquire resources.
      void
      onResourceAcquisition(void)
      {

      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
      }

      void
      proximity(void)
      {
        //if the vehicle is near to the way-point
        IMC::FollowRefState* msg;
        if (msg->proximity == IMC::FollowRefState::PROX_XY_NEAR)
        return;
      }

      void
      consume(const IMC::Abort* msg)
      {
        if (msg->getDestination() != getSystemId())
        return;

        war(DTR("Abort detected. Stop control..."));
        requestDeactivation();
      }


      void consume(const IMC::EstimatedState* msg)
      {
        float pi = 3.14159265359;

        if (msg->getSource() != getSystemId())
        return;

        m_estate = *msg;
        m_estate.lon = msg -> lon;
        m_estate.lat = msg -> lat;

        //calculate position according to WGS84
        m_estate.lat = m_estate.lat + (m_estate.x * 2 * pi)/40075000;
        m_estate.lon = m_estate.lon + (m_estate.y * 2 * pi)/(40075000 * cos(m_estate.lat));
        dispatch(m_estate);
      }

      void consume(const IMC::FollowRefState* msg)
      {
        if(msg->state == IMC::FollowRefState::FR_WAIT)
          war("Hello");

          m_ref.flags = Reference::FLAG_LOCATION;
          m_ref.lon = m_estate.lon;
          m_ref.lat = m_estate.lat;

          m_ref_state.reference.set(m_ref);
          m_ref_state.proximity = IMC::FollowRefState::PROX_XY_NEAR;
          m_ref_state.state = IMC::FollowRefState::FR_WAIT;
          m_ref_state.control_ent = msg->control_ent;
          m_ref_state.control_src = msg->control_src;
         dispatch(m_ref_state);
        //first reference position of estimated position
      }


      //! Main loop.
      void
      onMain(void)
      {//wait for 10 seconds
        DUNE::Time::Delay::waitNsec(m_args.waiting_time * 1000000000.0);
        // notice starting the follow reference
        war("Starting followref");

        IMC::PlanControl pc;
        pc.plan_id = "caravela_plan";
        pc.op = IMC::PlanControl::PC_START; //operation
        pc.type = IMC::PlanControl::PC_REQUEST; //type
        pc.flags = IMC::PlanControl::FLG_IGNORE_ERRORS;
        pc.request_id = 0;

        IMC::FollowReference man;
        man.control_src = 0xFFFF;
        man.control_ent = 0xFF;
        man.loiter_radius = 7.5;
        man.timeout = m_args.connection_timeout + 10;
        man.altitude_interval = 2.0; //

        IMC::PlanManeuver pm;
        pm.maneuver_id = "followref";
        pm.data.set(man);

        IMC::PlanSpecification ps;
        ps.plan_id = pc.plan_id;
        ps.start_man_id = pm.maneuver_id;
        ps.maneuvers.push_back(pm);
        pc.arg.set(ps);
        pc.flags = 0;
        pc.setDestination(m_ctx.resolver.id());

        dispatch(pc);

        DUNE::Time::Delay::waitNsec(1000000000.0);

        vector<double> lon_wp{m_args.h, m_args.h, m_estate.lon, m_estate.lon, m_args.h, m_args.h};
        vector<double> lat_wp{m_estate.lat, m_estate.lat + m_args.s, m_estate.lat + m_args.s, m_estate.lat + 2*m_args.s, m_estate.lat + 2*m_args.s, m_estate.lat + 3*m_args.s};



        for (double i = 0 ; i < 6 ; i++)
        {
            proximity(); // if the vehicle is near to the way-point
            m_ref.lon = lon_wp[i];
            m_ref.lat = lat_wp[i];
        }

      }


    };
  }
}

DUNE_TASK
