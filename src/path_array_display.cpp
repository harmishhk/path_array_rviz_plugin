/*
 * Original work Copyright (c) 2008, Willow Garage, Inc.
 * Modified work Copyright (c) 2016 LAAS/CNRS
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

#include <boost/bind.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreBillboardSet.h>
#include <OGRE/OgreMatrix4.h>

#include <tf/transform_listener.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/validate_floats.h>
#include <rviz/ogre_helpers/billboard_line.h>

#include "path_array_display.h"

namespace path_array_rviz_plugin
{
    PathArrayDisplay::PathArrayDisplay()
    {
        style_property_ = new rviz::EnumProperty( "Line Style", "Lines",
            "The rendering operation to use to draw the grid lines.",
            this, SLOT( updateStyle() ));

        style_property_->addOption( "Lines", LINES );
        style_property_->addOption( "Billboards", BILLBOARDS );

        line_width_property_ = new rviz::FloatProperty( "Line Width", 0.03,
            "The width, in meters, of each path line."
            "Only works with the 'Billboards' style.",
            this, SLOT( updateLineWidth() ), this );

        line_width_property_->setMin( 0.001 );
        line_width_property_->hide();

        color_property_ = new rviz::ColorProperty( "Color", QColor( 25, 255, 0 ),
            "Color to draw the path array.", this );

        alpha_property_ = new rviz::FloatProperty( "Alpha", 1.0,
            "Amount of transparency to apply to the path array.", this );

        buffer_length_property_ = new rviz::IntProperty( "Buffer Length", 1,
            "Number of path arrays to display.",
            this, SLOT( updateBufferLength() ));
        buffer_length_property_->setMin( 1 );

        offset_property_ = new rviz::VectorProperty( "Offset", Ogre::Vector3::ZERO,
            "Allows you to offset the path array from the origin of the reference frame.  In meters.",
            this, SLOT( updateOffset() ));
    }

    PathArrayDisplay::~PathArrayDisplay()
    {
        destroyObjects();
    }

    void PathArrayDisplay::onInitialize()
    {
        MFDClass::onInitialize();
        updateBufferLength();
    }

    void PathArrayDisplay::reset()
    {
        MFDClass::reset();
        updateBufferLength();
    }

    void PathArrayDisplay::updateStyle()
    {
        LineStyle style = (LineStyle) style_property_->getOptionInt();

        switch( style )
        {
        case LINES:
        default:
            line_width_property_->hide();
            break;

        case BILLBOARDS:
            line_width_property_->show();
            break;
        }

        updateBufferLength();
    }

    void PathArrayDisplay::updateLineWidth()
    {
        LineStyle style = (LineStyle) style_property_->getOptionInt();
        float line_width = line_width_property_->getFloat();

        if(style == BILLBOARDS) {
            for( size_t i = 0; i < billboard_lines_array_.size(); i++ )
            {
                for( size_t j = 0; j < billboard_lines_array_[ i ].size(); j++ )
                {
                    rviz::BillboardLine* billboard_line = billboard_lines_array_[ i ][ j ];
                    if( billboard_line ) billboard_line->setLineWidth( line_width );
                }
            }
        }
        context_->queueRender();
    }

    void PathArrayDisplay::updateOffset()
    {
        scene_node_->setPosition( offset_property_->getVector() );
        context_->queueRender();
    }

    void PathArrayDisplay::destroyObjects()
    {
        // Destroy all simple lines, if any
        for( size_t i = 0; i < manual_objects_array_.size(); i++ )
        {
            for( size_t j = 0; j < manual_objects_array_[ i ].size(); j++)
            {
                Ogre::ManualObject*& manual_object = manual_objects_array_[ i ][ j ];
                if( manual_object )
                {
                    manual_object->clear();
                    scene_manager_->destroyManualObject( manual_object );
                    manual_object = NULL; // ensure it doesn't get destroyed again
                }
            }
        }

        // Destroy all billboards, if any
        for( size_t i = 0; i < billboard_lines_array_.size(); i++ )
        {
            for( size_t j = 0; j < billboard_lines_array_[ i ].size(); j++)
            {
                rviz::BillboardLine*& billboard_line = billboard_lines_array_[ i ][ j ];
                if( billboard_line )
                {
                    delete billboard_line; // also destroys the corresponding scene node
                    billboard_line = NULL; // ensure it doesn't get destroyed again
                }
            }
        }
    }

    void PathArrayDisplay::updateBufferLength()
    {
        // Delete old path objects
        destroyObjects();

        // Read options
        int buffer_length = buffer_length_property_->getInt();
        LineStyle style = (LineStyle) style_property_->getOptionInt();

        // Create new path objects
        switch(style)
        {
        case LINES: // simple lines with fixed width of 1px
            manual_objects_array_.resize( buffer_length );
            // for( size_t i = 0; i < manual_objects_array_.size(); i++ )
            // {
            //     Ogre::ManualObject* manual_object = scene_manager_->createManualObject();
            //     manual_object->setDynamic( true );
            //     scene_node_->attachObject( manual_object );
            //
            //     manual_objects_[ i ] = manual_object;
            // }
            break;

        case BILLBOARDS: // billboards with configurable width
            billboard_lines_array_.resize( buffer_length );
            // for( size_t i = 0; i < billboard_lines_.size(); i++ )
            // {
            //     rviz::BillboardLine* billboard_line = new rviz::BillboardLine(scene_manager_, scene_node_);
            //     billboard_lines_[ i ] = billboard_line;
            // }
            break;
        }
    }

    void PathArrayDisplay::processMessage( const path_array_rviz_plugin::PathArray::ConstPtr& msg )
    {
        // Calculate index of oldest element in cyclic buffer
        size_t bufferIndex = messages_received_ % buffer_length_property_->getInt();

        LineStyle style = (LineStyle) style_property_->getOptionInt();
        // Ogre::ManualObject* manual_object = NULL;
        // rviz::BillboardLine* billboard_line = NULL;

        // Delete oldest element
        switch(style)
        {
        case LINES:
            // manual_object = manual_objects_array_[ bufferIndex ];
            // manual_object->clear();
            for( auto manual_object : manual_objects_array_[ bufferIndex ] )
            {
                if( manual_object )
                {
                    manual_object->clear();
                    scene_manager_->destroyManualObject( manual_object );
                    manual_object = NULL; // ensure it doesn't get destroyed again
                }
            }
            manual_objects_array_[ bufferIndex ].clear();
            break;

        case BILLBOARDS:
            // billboard_line = billboard_lines_[ bufferIndex ];
            // billboard_line->clear();
            for( auto billboard_line : billboard_lines_array_[ bufferIndex ] )
            {
                if( billboard_line )
                {
                    delete billboard_line; // also destroys the corresponding scene node
                    billboard_line = NULL; // ensure it doesn't get destroyed again
                }
            }
            billboard_lines_array_[ bufferIndex ].clear();
            break;
        }

        Ogre::ColourValue color = color_property_->getOgreColor();
        color.a = alpha_property_->getFloat();

        Ogre::Vector3 position;
        Ogre::Quaternion orientation;

        // Check if path array contains invalid coordinate values
        for(auto path : msg->paths)
        {
            if( !rviz::validateFloats( path.poses ))
            {
                setStatus( rviz::StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)" );
                return;
            }

            // Lookup transform into fixed frame
            if( !context_->getFrameManager()->getTransform( path.header, position, orientation ))
            {
                ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", path.header.frame_id.c_str(), qPrintable( fixed_frame_ ));
            }

            Ogre::Matrix4 transform( orientation );
            transform.setTrans( position );

            //  scene_node_->setPosition( position );
            //  scene_node_->setOrientation( orientation );

            uint32_t num_points = path.poses.size();
            float line_width = line_width_property_->getFloat();

            switch(style)
            {
            case LINES:
                {
                    Ogre::ManualObject* manual_object = scene_manager_->createManualObject();
                    manual_object->setDynamic( true );
                    scene_node_->attachObject( manual_object );
                    manual_objects_array_[ bufferIndex ].push_back( manual_object );

                    manual_object->estimateVertexCount( num_points );
                    manual_object->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP );
                    for( uint32_t i=0; i < num_points; ++i )
                    {
                        const geometry_msgs::Point& pos = path.poses[ i ].pose.position;
                        Ogre::Vector3 xpos = transform * Ogre::Vector3( pos.x, pos.y, pos.z );
                        manual_object->position( xpos.x, xpos.y, xpos.z );
                        manual_object->colour( color );
                    }
                    manual_object->end();
                    break;
                }

            case BILLBOARDS:
                {
                    rviz::BillboardLine* billboard_line = new rviz::BillboardLine(scene_manager_, scene_node_);
                    billboard_lines_array_[ bufferIndex ].push_back(billboard_line);

                    billboard_line->setNumLines( 1 );
                    billboard_line->setMaxPointsPerLine( num_points );
                    billboard_line->setLineWidth( line_width );
                    for( uint32_t i=0; i < num_points; ++i)
                    {
                        const geometry_msgs::Point& pos = path.poses[ i ].pose.position;
                        Ogre::Vector3 xpos = transform * Ogre::Vector3( pos.x, pos.y, pos.z );
                        billboard_line->addPoint( xpos, color );
                    }
                    break;
                }
            }
        }
    }
} // namespace path_array_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( path_array_rviz_plugin::PathArrayDisplay, rviz::Display )
