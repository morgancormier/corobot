/*
 * Copyright (c) 2009, CoroWare
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Stanford U. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <corobot_msgs/GPSFix.h>
#include <corobot_msgs/GPSStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <libgpsmm.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

/** 
 * Node interfacing with a usb gps sensor
 */

using namespace corobot_msgs;
using namespace sensor_msgs;

// Class interfacing with the gps sensor
class GPSDClient {
  public:
  
    GPSDClient() : privnode("~"), use_gps_time(true) {
#if GPSD_API_MAJOR_VERSION < 5
    gps = new gpsmm();
    gps_state = 0;
#endif
    }

    ~GPSDClient()
    {
	    delete gps;
    }

    // Start the gps sensor
    bool start() {
      // Advertise the topics
      gps_fix_pub = node.advertise<GPSFix>("extended_fix", 1);
      navsat_fix_pub = node.advertise<NavSatFix>("fix", 1);

      privnode.getParam("use_gps_time", use_gps_time);

      // Set up where the gps data are read from
      std::string host = "localhost";
      int port = 2947;
      privnode.getParam("host", host);
      privnode.getParam("port", port);

      char port_s[12];
      snprintf(port_s, 12, "%d", port);

      // Start gpsmm 
      gps_data_t *resp;
#if GPSD_API_MAJOR_VERSION >= 5
      gps = new gpsmm(host.c_str(), port_s);
#elif GPSD_API_MAJOR_VERSION < 5
      resp = gps->open(host.c_str(), port_s);
      if (resp == NULL) {
        ROS_ERROR("Failed to open GPSd");
	      gps_state = 1;
        return false;
      }
#endif

      // Start the stream that will give us gps values
#if GPSD_API_MAJOR_VERSION >= 5
      resp = gps->stream(WATCH_ENABLE);
      if (resp == NULL) {
        ROS_ERROR("Failed to intialize the gps");
	    gps_state = 1;
	    return false;
      }
#elif GPSD_API_MAJOR_VERSION == 4
      resp = gps->stream(WATCH_ENABLE);
      if (resp == NULL) {
        ROS_ERROR("Failed to intialize the gps");
	    gps_state = 1;
	    return false;
      }
#elif GPSD_API_MAJOR_VERSION == 3
      gps->query("w\n");
#else
#error "gpsd_client only supports gpsd API versions 3, 4 and 5"
      gps_state = 2;
#endif
      

      ROS_INFO("GPSd opened");
      return true;
    }

    // Read the gps data and process them
    void step() {

      gps_data_t *p;
#if GPSD_API_MAJOR_VERSION >= 5
      p = gps->read();
#elif GPSD_API_MAJOR_VERSION < 5
      p = gps->poll();
#endif
      process_data(p);
    }

    // Stop the gps
    void stop() {
      // gpsmm doesn't have a close method? OK ...
    }

    /**
     * Function that will report the status of the hardware to the diagnostic topic
     */
    void gps_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
	    if (gps_state == 0)  
		    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "The gps is working");
	    else if (gps_state == 1)
	    {
		    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Can't intialize");
		    stat.addf("Recommendation", "The gps could not be initialized. Please make sure the gps is connected to the motherboard and is configured. You can follow points 1.3 and after in the following tutorial for the configuration: http://ros.org/wiki/gpsd_client/Tutorials/Getting started with gpsd_client");
	    }
	    else if (gps_state == 2)
	    {
		    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "wrong lib gpsd version");
		    stat.addf("Recommendation", "Please make sure that gpsd is installed and that you have at least the api major version 3 or after installed.");
	    }
    }

  private:
    ros::NodeHandle node;
    ros::NodeHandle privnode;
    // ROS publisher
    ros::Publisher gps_fix_pub, navsat_fix_pub;
    gpsmm *gps;
    int gps_state;

    bool use_gps_time;

    void process_data(struct gps_data_t* p) 
    {
      if (p == NULL)
        return;

      if (!p->online)
        return;

      process_data_gps(p);
      process_data_navsat(p);
    }

#if GPSD_API_MAJOR_VERSION == 5
#define SATS_VISIBLE p->satellites_visible
#elif GPSD_API_MAJOR_VERSION == 4
#define SATS_VISIBLE p->satellites_visible
#elif GPSD_API_MAJOR_VERSION == 3
#define SATS_VISIBLE p->satellites
#endif

    // process the data received from the gps
    void process_data_gps(struct gps_data_t* p) {
      ros::Time time = ros::Time::now();

      GPSFix fix;
      GPSStatus status;

      status.header.stamp = time;
      fix.header.stamp = time;

      status.satellites_used = p->satellites_used;

      status.satellite_used_prn.resize(status.satellites_used);
      for (int i = 0; i < status.satellites_used; ++i) {
        status.satellite_used_prn[i] = p->used[i];
      }

      status.satellites_visible = SATS_VISIBLE;

      status.satellite_visible_prn.resize(status.satellites_visible);
      status.satellite_visible_z.resize(status.satellites_visible);
      status.satellite_visible_azimuth.resize(status.satellites_visible);
      status.satellite_visible_snr.resize(status.satellites_visible);

      for (int i = 0; i < SATS_VISIBLE; ++i) {
        status.satellite_visible_prn[i] = p->PRN[i];
        status.satellite_visible_z[i] = p->elevation[i];
        status.satellite_visible_azimuth[i] = p->azimuth[i];
    //    status.satellite_visible_snr[i] = p->ss[i];
      }

      if ((p->status & STATUS_FIX) && !isnan(p->fix.epx)) {
        // FIXME: gpsmm puts its constants in the global namespace, so `GPSStatus::STATUS_FIX' is illegal.
        status.status = 0; 

        if (p->status & STATUS_DGPS_FIX)
        {
          // same here
          status.status |= 18; 
        }

        // Set the important values into the gps variables
        fix.time = p->fix.time;
        fix.latitude = p->fix.latitude;
        fix.longitude = p->fix.longitude;
        fix.altitude = p->fix.altitude;
        fix.track = p->fix.track;
        fix.speed = p->fix.speed;
        fix.climb = p->fix.climb;

#if GPSD_API_MAJOR_VERSION > 3
        fix.pdop = p->dop.pdop;
        fix.hdop = p->dop.hdop;
        fix.vdop = p->dop.vdop;
        fix.tdop = p->dop.tdop;
        fix.gdop = p->dop.gdop;
#else
        fix.pdop = p->pdop;
        fix.hdop = p->hdop;
        fix.vdop = p->vdop;
        fix.tdop = p->tdop;
        fix.gdop = p->gdop;
#endif

        fix.err = p->epe;
        fix.err_vert = p->fix.epv;
        fix.err_track = p->fix.epd;
        fix.err_speed = p->fix.eps;
        fix.err_climb = p->fix.epc;
        fix.err_time = p->fix.ept;

        /* TODO: attitude */
      } else {
      	status.status = -1; // STATUS_NO_FIX
      }

      fix.status = status;

      gps_fix_pub.publish(fix);
    }

    void process_data_navsat(struct gps_data_t* p) {
      NavSatFixPtr fix(new NavSatFix);

      /* TODO: Support SBAS and other GBAS. */

      if (use_gps_time)
        fix->header.stamp = ros::Time(p->fix.time);
      else
        fix->header.stamp = ros::Time::now();

      /* gpsmm pollutes the global namespace with STATUS_,
       * so we need to use the ROS message's integer values
       * for status.status
       */
      switch (p->status) {
        case STATUS_NO_FIX:
          // NavSatStatus::STATUS_NO_FIX;
          fix->status.status = -1; 
          break;
        case STATUS_FIX:
          // NavSatStatus::STATUS_FIX;
          fix->status.status = 0; 
          break;
        case STATUS_DGPS_FIX:
          // NavSatStatus::STATUS_GBAS_FIX;
          fix->status.status = 2; 
          break;
      }

      fix->status.service = NavSatStatus::SERVICE_GPS;

      fix->latitude = p->fix.latitude;
      fix->longitude = p->fix.longitude;
      fix->altitude = p->fix.altitude;

      /* gpsd reports status=OK even when there is no current fix,
       * as long as there has been a fix previously. Throw out these
       * fake results, which have NaN variance
       */
      if (isnan(p->fix.epx)) {
        //return;    //MODIFIED
      }

      fix->position_covariance[0] = p->fix.epx;
      fix->position_covariance[4] = p->fix.epy;
      fix->position_covariance[8] = p->fix.epv;

      fix->position_covariance_type = NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

      navsat_fix_pub.publish(fix);
    }
};

int main(int argc, char ** argv) {
  ros::init(argc, argv, "corobot_gps");

  GPSDClient client;


  //create an updater that will send information on the diagnostics topics
  diagnostic_updater::Updater updater;
  updater.setHardwareIDf("GPS");
  updater.add("gps", &client, &GPSDClient::gps_diagnostic); //function that will be executed with updater.update()

  if (!client.start())
  {
    updater.force_update();
    return -1;
  }


  while(ros::ok()) {
    ros::spinOnce();
    updater.update();
    client.step();
  }

  client.stop();
}
