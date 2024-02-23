/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef OV_CORE_TRACK_APRIL_H
#define OV_CORE_TRACK_APRIL_H

#if ENABLE_APRIL_TAGS
#include "utils/apriltag/apriltag.h"
#include "utils/apriltag/tag36h11.h"
#include "utils/apriltag/tag25h9.h"
#include "utils/apriltag/tag16h5.h"
#include "utils/apriltag/tagCircle21h7.h"
#include "utils/apriltag/tagCircle49h12.h"
#include "utils/apriltag/tagCustom48h12.h"
#include "utils/apriltag/tagStandard41h12.h"
#include "utils/apriltag/tagStandard52h13.h"
#endif

#include "TrackBase.h"

namespace ov_core {

/**
 * @brief Tracking of April tags.
 *
 * This class handles the tracking of Apriltag(https://github.com/AprilRobotics/apriltag)
 * 
 */
class TrackApril : public TrackBase {

public:
  /**
   * @brief Public constructor with configuration variables
   * @param cameras camera calibration object which has all camera intrinsics in it
   * @param numapril the max id of the april tags, we don't use any tags greater than this value even if we extract them
   * @param stereo if we should do stereo feature tracking or binocular
   * @param histmethod what type of histogram pre-processing should be done (histogram eq?)
   * @param downsize we can scale the image by 1/2 to increase Aprilz tag extraction speed
   */
  explicit TrackApril(std::unordered_map<size_t, std::shared_ptr<CamBase>> cameras, std::string family, bool stereo, HistogramMethod histmethod,
                      bool downsize)
      : TrackBase(cameras, 0, 1000, stereo, histmethod), family_name(family), do_downsizing(downsize) {
#if ENABLE_APRIL_TAGS
    if (family_name == "tag36h11") {
      tag_family = tag36h11_create();
    } else if (family_name == "tag25h9") {
      tag_family = tag25h9_create();
    } else if (family_name == "tag16h5") {
      tag_family = tag16h5_create();
    } else if (family_name == "tagCircle21h7") {
      tag_family = tagCircle21h7_create();
    } else if (family_name == "tagCircle49h12") {
      tag_family = tagCircle49h12_create();
    } else if (family_name == "tagCustom48h12") {
      tag_family = tagCustom48h12_create();
    } else if (family_name == "tagStandard41h12") {
      tag_family = tagStandard41h12_create();
    } else if (family_name == "tagStandard52h13") {
      tag_family = tagStandard52h13_create();
    }
    tag_detector = apriltag_detector_create();
    apriltag_detector_add_family(tag_detector, tag_family);
    tag_detector->quad_decimate  = 1.0;
    tag_detector->quad_sigma     = 0.0;
    tag_detector->nthreads       = 2;
    tag_detector->debug          = 0;
    tag_detector->refine_edges   = 1;

    // max_tag_id = tag_family->ncodes;
    max_tag_id = 1000;
    // currid = 4 * (size_t)max_tag_id + 1;  // re-assign 
#else
    PRINT_ERROR(RED "[ERROR]: you have not compiled with april tag support!!!\n" RESET);
    std::exit(EXIT_FAILURE);
#endif
  }

  ~TrackApril() {
#if ENABLE_APRIL_TAGS
    apriltag_detector_destroy(tag_detector);
    if (family_name == "tag36h11") {
      tag36h11_destroy(tag_family);
    } else if (family_name == "tag25h9") {
      tag25h9_destroy(tag_family);
    } else if (family_name == "tag16h5") {
      tag16h5_destroy(tag_family);
    } else if (family_name == "tagCircle21h7") {
      tagCircle21h7_destroy(tag_family);
    } else if (family_name == "tagCircle49h12") {
      tagCircle49h12_destroy(tag_family);
    } else if (family_name == "tagCustom48h12") {
      tagCustom48h12_destroy(tag_family);
    } else if (family_name == "tagStandard41h12") {
      tagStandard41h12_destroy(tag_family);
    } else if (family_name == "tagStandard52h13") {
      tagStandard52h13_destroy(tag_family);
    }
#endif
  }

  /**
   * @brief Process a new image
   * @param message Contains our timestamp, images, and camera ids
   */
  void feed_new_camera(const CameraData &message) override;

#if ENABLE_APRIL_TAGS
  /**
   * @brief We override the display equation so we can show the tags we extract.
   * @param img_out image to which we will overlayed features on
   * @param r1,g1,b1 first color to draw in
   * @param r2,g2,b2 second color to draw in
   * @param overlay Text overlay to replace to normal "cam0" in the top left of screen
   */
  void display_active(cv::Mat &img_out, int r1, int g1, int b1, int r2, int g2, int b2, std::string overlay = "") override;
#endif

protected:
#if ENABLE_APRIL_TAGS
  /**
   * @brief Process a new monocular image
   * @param timestamp timestamp the new image occurred at
   * @param imgin new cv:Mat grayscale image
   * @param cam_id the camera id that this new image corresponds too
   * @param maskin tracking mask for the given input image
   */
  void perform_tracking(double timestamp, const cv::Mat &imgin, size_t cam_id, const cv::Mat &maskin);
#endif

  // Max tag ID we should extract from (i.e., number of april tags starting from zero)
  int max_tag_id;

  // If we should downsize the image
  bool do_downsizing;

  // About April Tag
  std::string family_name;

#if ENABLE_APRIL_TAGS
  apriltag_family_t *tag_family;
  apriltag_detector_t *tag_detector;

  // Our tag IDs and corner we will get from the extractor
  std::unordered_map<size_t, std::vector<int>> ids_april;
  std::unordered_map<size_t, std::vector<std::vector<cv::Point2f>>> corners, rejects;
  std::unordered_map<size_t, std::vector<cv::Point2f>> centers;
#endif
};

} // namespace ov_core

#endif /* OV_CORE_TRACK_APRIL_H */