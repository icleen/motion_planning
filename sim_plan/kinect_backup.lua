
function sysCall_init()
    -- The child script initialization

    --kinectInfo=sim.getObjectHandle('kinect_depth')
    kinectRGB=sim.getObjectHandle('kinect_rgb')
    colorView=sim.floatingViewAdd(0.69,0.9,0.2,0.2,0)
    sim.adjustView(colorView,kinectRGB,64)

    kinectDepth=sim.getObjectHandle('kinect_depth')
    depthView=sim.floatingViewAdd(0.9,0.9,0.2,0.2,0)
    sim.adjustView(depthView,kinectDepth,64)

    objectHandle=sim.getObjectAssociatedWithScript(sim.handle_self)
    objectName=sim.getObjectName(objectHandle)

    -- Prepare the float32 publisher and subscriber (we subscribe to the topic we advertise):
    if simROS then
        --info_pub=simROS.advertise('/camera/camera_info','sensor_msgs/CameraInfo')
        --info_pub2=simROS.advertise('/camera/rgb/camera_info','sensor_msgs/CameraInfo')

        rgb_pub=simROS.advertise('/camera/rgb/image_rect_color','sensor_msgs/Image')
        simROS.publisherTreatUInt8ArrayAsString(rgb_pub)

        depth_pub=simROS.advertise('/camera/depth/image', 'sensor_msgs/Image')
        simROS.publisherTreatUInt8ArrayAsString(depth_pub)
    else
        print("<font color='#F00'>ROS interface was not found. Cannot run.</font>@html")
    end
end

function sysCall_sensing()
    if simROS then
        --local data=sim.getVisionSensor(kinectInfo)

        -- Publish the image of the active vision sensor:
        local data,width,height=sim.getVisionSensorCharImage(kinectRGB)
        rgb_d={}
        rgb_d['header'] = {stamp=simROS.getTime(), frame_id="rgb_mert"}
        rgb_d['height'] = height
        rgb_d['width'] = width
        rgb_d['encoding'] = 'rgb8'
        rgb_d['is_bigendian'] = 1 --information on where the important side is
        rgb_d['step'] = width * 3 -- channels r,g,b
        rgb_d['data'] = data
        simROS.publish(rgb_pub,rgb_d)

        -- QRP_start
        local data=sim.getVisionSensorDepthBuffer(kinectDepth+sim.handleflag_codedstring)
        local res,nearClippingPlane=sim.getObjectFloatParameter(kinectDepth,sim.visionfloatparam_near_clipping)
        local res,farClippingPlane=sim.getObjectFloatParameter(kinectDepth,sim.visionfloatparam_far_clipping)
        nearClippingPlane=nearClippingPlane*1000 -- we want mm
        farClippingPlane=farClippingPlane*1000 -- we want mm
        data=sim.transformBuffer(data,sim.buffer_float,farClippingPlane-nearClippingPlane,nearClippingPlane,sim.buffer_uint16)
        local res=sim.getVisionSensorResolution(kinectDepth)
        width, height = res[1], res[2]
        depth_d={}
        depth_d['header'] = {stamp=simROS.getTime(), frame_id="depth_mert"}
        depth_d['height'] = height
        depth_d['width'] = width
        depth_d['encoding'] = '16UC1'
        depth_d['is_bigendian'] = 1
        depth_d['step'] = width*2
        depth_d['data'] = data
        simROS.publish(depth_pub,depth_d)

        --QRP_end
   end
end

function sysCall_cleanup()
    -- Following not really needed in a simulation script (i.e. automatically shut down at simulation end):
    if rosInterfacePresent then
        simROS.shutdownPublisher(publisher)
        simROS.shutdownSubscriber(subscriber)
    end
end
