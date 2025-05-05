/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VioBackend-definitions.h
 * @brief  Definitions for VioBackend.
 * @author Antoni Rosinol
 */

 #pragma once
 
 #include <glog/logging.h>
 
 namespace VIO {
 
 enum class GnssBackendModality {
   //! Only use structureless factors, equiv to normal Vio.
//    STRUCTURELESS = 0,
   STRUCTURELESS_WITH_GNSS = 0
//    //! Converts all structureless factors to projection factors
//    PROJECTION = 1,
//    //! Projection factors used for regularities.
//    STRUCTURELESS_AND_PROJECTION = 2,
//    //! Projection Vio + regularity factors.
//    PROJECTION_AND_REGULARITY = 3,
//    //! All types of factors used.
//    STRUCTURELESS_PROJECTION_AND_REGULARITY = 4
 };
 
 }  // namespace VIO
 