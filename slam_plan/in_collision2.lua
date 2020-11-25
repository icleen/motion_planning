function sysCall_init()

    wall=sim.getObjectHandle('80cmHighWall100cm_visible')
    wall0=sim.getObjectHandle('80cmHighWall100cm_visible0')
    wall1=sim.getObjectHandle('80cmHighWall100cm_visible1')
    wall2=sim.getObjectHandle('80cmHighWall100cm_visible2')
    wall3=sim.getObjectHandle('80cmHighWall50cm_visible11')
    wall4=sim.getObjectHandle('80cmHighWall100cm_visible4')
    wall5=sim.getObjectHandle('80cmHighWall100cm_visible5')
    wall6=sim.getObjectHandle('80cmHighWall100cm_visible6')
    wall7=sim.getObjectHandle('80cmHighWall100cm_visible7')
    wall8=sim.getObjectHandle('80cmHighWall100cm_visible8')

    wall9=sim.getObjectHandle('80cmHighWall50cm_visible')
    wall10=sim.getObjectHandle('80cmHighWall50cm_visible0')
    wall11=sim.getObjectHandle('80cmHighWall50cm_visible1')
    wall12=sim.getObjectHandle('80cmHighWall50cm_visible2')
    wall13=sim.getObjectHandle('80cmHighWall50cm_visible3')
    wall14=sim.getObjectHandle('80cmHighWall50cm_visible4')
    wall15=sim.getObjectHandle('80cmHighWall50cm_visible5')
    wall16=sim.getObjectHandle('80cmHighWall50cm_visible6')
    wall17=sim.getObjectHandle('80cmHighWall50cm_visible7')
    wall18=sim.getObjectHandle('80cmHighWall50cm_visible8')
    wall19=sim.getObjectHandle('80cmHighWall50cm_visible9')
    wall20=sim.getObjectHandle('80cmHighWall50cm_visible10')

    wall21=sim.getObjectHandle('80cmHighWall200cm_visible')
    wall22=sim.getObjectHandle('80cmHighWall200cm_visible0')
    wall23=sim.getObjectHandle('80cmHighWall200cm_visible1')
    wall24=sim.getObjectHandle('80cmHighWall200cm_visible2')
    wall25=sim.getObjectHandle('80cmHighWall200cm_visible3')
    wall26=sim.getObjectHandle('80cmHighWall200cm_visible4')
    wall27=sim.getObjectHandle('80cmHighWall200cm_visible5')
    wall28=sim.getObjectHandle('80cmHighWall200cm_visible6')
    wall29=sim.getObjectHandle('80cmHighWall200cm_visible7')
    wall30=sim.getObjectHandle('80cmHighWall200cm_visible8')
    wall31=sim.getObjectHandle('80cmHighWall200cm_visible9')
    wall32=sim.getObjectHandle('80cmHighWall200cm_visible10')
    wall33=sim.getObjectHandle('80cmHighWall200cm_visible11')
    wall34=sim.getObjectHandle('80cmHighWall200cm_visible12')
    wall35=sim.getObjectHandle('80cmHighWall200cm_visible13')
    wall36=sim.getObjectHandle('80cmHighWall200cm_visible14')
    wall37=sim.getObjectHandle('80cmHighWall200cm_visible15')
    wall38=sim.getObjectHandle('80cmHighWall200cm_visible16')
    wall39=sim.getObjectHandle('80cmHighWall200cm_visible17')
    wall40=sim.getObjectHandle('80cmHighWall200cm_visible18')
    wall41=sim.getObjectHandle('80cmHighWall200cm_visible19')
    wall42=sim.getObjectHandle('80cmHighWall200cm_visible20')
  	width=1.50
    length=2.355

    simRemoteApi.start(19997)
end

CollisionDetection=function(inInts,inFloats,inStrings,inBuffer)

  local posx = inFloats[1]
	local posy = inFloats[2]
	local posz = inFloats[3]

  local wheel_rl = sim.createDummy(0.1)
	local errorReportMode=sim.getInt32Parameter(sim.intparam_error_report_mode)
	sim.setInt32Parameter(sim.intparam_error_report_mode,0) -- temporarily suppress error output (because we are not allowed to have two times the same object name)
	sim.setObjectName(wheel_rl,'wheel_rl')
	sim.setInt32Parameter(sim.intparam_error_report_mode,errorReportMode) -- restore the original error report mode
	sim.setObjectPosition(wheel_rl, inInts[1], {posx-width, posy-length, posz})

	local wheel_rr = sim.createDummy(0.1)
	errorReportMode=sim.getInt32Parameter(sim.intparam_error_report_mode)
	sim.setInt32Parameter(sim.intparam_error_report_mode,0) -- temporarily suppress error output (because we are not allowed to have two times the same object name)
	sim.setObjectName(wheel_rr,'wheel_rr')
	sim.setInt32Parameter(sim.intparam_error_report_mode,errorReportMode) -- restore the original error report mode
	sim.setObjectPosition(wheel_rr, inInts[1], {posx+width, posy-length, posz})

	local wheel_fl = sim.createDummy(0.1)
	errorReportMode=sim.getInt32Parameter(sim.intparam_error_report_mode)
	sim.setInt32Parameter(sim.intparam_error_report_mode,0) -- temporarily suppress error output (because we are not allowed to have two times the same object name)
	sim.setObjectName(wheel_fl,'wheel_fl')
	sim.setInt32Parameter(sim.intparam_error_report_mode,errorReportMode) -- restore the original error report mode
	sim.setObjectPosition(wheel_fl, inInts[1], {posx-width, posy+length, posz})

	local wheel_fr = sim.createDummy(0.1)
	errorReportMode=sim.getInt32Parameter(sim.intparam_error_report_mode)
	sim.setInt32Parameter(sim.intparam_error_report_mode,0) -- temporarily suppress error output (because we are not allowed to have two times the same object name)
	sim.setObjectName(wheel_fr,'wheel_fr')
	sim.setInt32Parameter(sim.intparam_error_report_mode,errorReportMode) -- restore the original error report mode
	sim.setObjectPosition(wheel_fr, inInts[1], {posx+width, posy+length, posz})

  local center = sim.createDummy(width)
	errorReportMode=sim.getInt32Parameter(sim.intparam_error_report_mode)
	sim.setInt32Parameter(sim.intparam_error_report_mode,0) -- temporarily suppress error output (because we are not allowed to have two times the same object name)
	sim.setObjectName(center,'center')
	sim.setInt32Parameter(sim.intparam_error_report_mode,errorReportMode) -- restore the original error report mode
	sim.setObjectPosition(center, -1, {posx, posy, posz})

    local colls = 0
    local count = 0
    local collsRed =0
    local collsBlue =0
    local collsGreen =0
    local collisionNumber = 1
    local collisionR  = 0
    local collisionB  = 0
    local collisionG  = 0
    local timeFlag = 0

    if ((sim.checkCollision(wheel_rr,wall) ==1) or (sim.checkCollision(wheel_rl,wall) ==1) or (sim.checkCollision(wheel_fr,wall) ==1) or (sim.checkCollision(wheel_fl,wall) ==1) or (sim.checkCollision(center,wall) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall0) ==1) or (sim.checkCollision(wheel_rl,wall0) ==1) or (sim.checkCollision(wheel_fr,wall0) ==1) or (sim.checkCollision(wheel_fl,wall0) ==1) or (sim.checkCollision(center,wall0) ==1)) then
          colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall1) ==1) or (sim.checkCollision(wheel_rl,wall1) ==1) or (sim.checkCollision(wheel_fr,wall1) ==1) or (sim.checkCollision(wheel_fl,wall1) ==1) or (sim.checkCollision(center,wall1) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall2) ==1) or (sim.checkCollision(wheel_rl,wall2) ==1) or (sim.checkCollision(wheel_fr,wall2) ==1) or (sim.checkCollision(wheel_fl,wall2) ==1) or (sim.checkCollision(center,wall2) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall3) ==1) or (sim.checkCollision(wheel_rl,wall3) ==1) or (sim.checkCollision(wheel_fr,wall3) ==1) or (sim.checkCollision(wheel_fl,wall3) ==1) or (sim.checkCollision(center,wall3) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall4) ==1) or (sim.checkCollision(wheel_rl,wall4) ==1) or (sim.checkCollision(wheel_fr,wall4) ==1) or (sim.checkCollision(wheel_fl,wall4) ==1) or (sim.checkCollision(center,wall4) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall5) ==1) or (sim.checkCollision(wheel_rl,wall5) ==1) or (sim.checkCollision(wheel_fr,wall5) ==1) or (sim.checkCollision(wheel_fl,wall5) ==1) or (sim.checkCollision(center,wall5) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall6) ==1) or (sim.checkCollision(wheel_rl,wall6) ==1) or (sim.checkCollision(wheel_fr,wall6) ==1) or (sim.checkCollision(wheel_fl,wall6) ==1) or (sim.checkCollision(center,wall6) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall7) ==1) or (sim.checkCollision(wheel_rl,wall7) ==1) or (sim.checkCollision(wheel_fr,wall7) ==1) or (sim.checkCollision(wheel_fl,wall7) ==1) or (sim.checkCollision(center,wall7) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall8) ==1) or (sim.checkCollision(wheel_rl,wall8) ==1) or (sim.checkCollision(wheel_fr,wall8) ==1) or (sim.checkCollision(wheel_fl,wall8) ==1) or (sim.checkCollision(center,wall8) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall9) ==1) or (sim.checkCollision(wheel_rl,wall9) ==1) or (sim.checkCollision(wheel_fr,wall9) ==1) or (sim.checkCollision(wheel_fl,wall9) ==1) or (sim.checkCollision(center,wall9) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall10) ==1) or (sim.checkCollision(wheel_rl,wall10) ==1) or (sim.checkCollision(wheel_fr,wall10) ==1) or (sim.checkCollision(wheel_fl,wall10) ==1) or (sim.checkCollision(center,wall10) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall11) ==1) or (sim.checkCollision(wheel_rl,wall11) ==1) or (sim.checkCollision(wheel_fr,wall11) ==1) or (sim.checkCollision(wheel_fl,wall11) ==1) or (sim.checkCollision(center,wall11) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall12) ==1) or (sim.checkCollision(wheel_rl,wall12) ==1) or (sim.checkCollision(wheel_fr,wall12) ==1) or (sim.checkCollision(wheel_fl,wall12) ==1) or (sim.checkCollision(center,wall12) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall13) ==1) or (sim.checkCollision(wheel_rl,wall13) ==1) or (sim.checkCollision(wheel_fr,wall13) ==1) or (sim.checkCollision(wheel_fl,wall13) ==1) or (sim.checkCollision(center,wall13) ==1)) then
     colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall14) ==1) or (sim.checkCollision(wheel_rl,wall14) ==1) or (sim.checkCollision(wheel_fr,wall14) ==1) or (sim.checkCollision(wheel_fl,wall14) ==1) or (sim.checkCollision(center,wall14) ==1)) then
          colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall15) ==1) or (sim.checkCollision(wheel_rl,wall15) ==1) or (sim.checkCollision(wheel_fr,wall15) ==1) or (sim.checkCollision(wheel_fl,wall15) ==1) or (sim.checkCollision(center,wall15) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall16) ==1) or (sim.checkCollision(wheel_rl,wall16) ==1) or (sim.checkCollision(wheel_fr,wall16) ==1) or (sim.checkCollision(wheel_fl,wall16) ==1) or (sim.checkCollision(center,wall16) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall17) ==1) or (sim.checkCollision(wheel_rl,wall17) ==1) or (sim.checkCollision(wheel_fr,wall17) ==1) or (sim.checkCollision(wheel_fl,wall17) ==1) or (sim.checkCollision(center,wall17) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall18) ==1) or (sim.checkCollision(wheel_rl,wall18) ==1) or (sim.checkCollision(wheel_fr,wall18) ==1) or (sim.checkCollision(wheel_fl,wall18) ==1) or (sim.checkCollision(center,wall18) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall19) ==1) or (sim.checkCollision(wheel_rl,wall19) ==1) or (sim.checkCollision(wheel_fr,wall19) ==1) or (sim.checkCollision(wheel_fl,wall19) ==1) or (sim.checkCollision(center,wall19) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall20) ==1) or (sim.checkCollision(wheel_rl,wall20) ==1) or (sim.checkCollision(wheel_fr,wall20) ==1) or (sim.checkCollision(wheel_fl,wall20) ==1) or (sim.checkCollision(center,wall20) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall21) ==1) or (sim.checkCollision(wheel_rl,wall21) ==1) or (sim.checkCollision(wheel_fr,wall21) ==1) or (sim.checkCollision(wheel_fl,wall21) ==1) or (sim.checkCollision(center,wall21) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall22) ==1) or (sim.checkCollision(wheel_rl,wall22) ==1) or (sim.checkCollision(wheel_fr,wall22) ==1) or (sim.checkCollision(wheel_fl,wall22) ==1) or (sim.checkCollision(center,wall22) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall23) ==1) or (sim.checkCollision(wheel_rl,wall23) ==1) or (sim.checkCollision(wheel_fr,wall23) ==1) or (sim.checkCollision(wheel_fl,wall23) ==1) or (sim.checkCollision(center,wall23) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall24) ==1) or (sim.checkCollision(wheel_rl,wall24) ==1) or (sim.checkCollision(wheel_fr,wall24) ==1) or (sim.checkCollision(wheel_fl,wall24) ==1) or (sim.checkCollision(center,wall24) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall25) ==1) or (sim.checkCollision(wheel_rl,wall25) ==1) or (sim.checkCollision(wheel_fr,wall25) ==1) or (sim.checkCollision(wheel_fl,wall25) ==1) or (sim.checkCollision(center,wall25) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall26) ==1) or (sim.checkCollision(wheel_rl,wall26) ==1) or (sim.checkCollision(wheel_fr,wall26) ==1) or (sim.checkCollision(wheel_fl,wall26) ==1) or (sim.checkCollision(center,wall26) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall27) ==1) or (sim.checkCollision(wheel_rl,wall27) ==1) or (sim.checkCollision(wheel_fr,wall27) ==1) or (sim.checkCollision(wheel_fl,wall27) ==1) or (sim.checkCollision(center,wall27) ==1)) then
     colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall28) ==1) or (sim.checkCollision(wheel_rl,wall28) ==1) or (sim.checkCollision(wheel_fr,wall28) ==1) or (sim.checkCollision(wheel_fl,wall28) ==1) or (sim.checkCollision(center,wall28) ==1)) then
          colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall29) ==1) or (sim.checkCollision(wheel_rl,wall29) ==1) or (sim.checkCollision(wheel_fr,wall29) ==1) or (sim.checkCollision(wheel_fl,wall29) ==1) or (sim.checkCollision(center,wall29) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall30) ==1) or (sim.checkCollision(wheel_rl,wall30) ==1) or (sim.checkCollision(wheel_fr,wall30) ==1) or (sim.checkCollision(wheel_fl,wall30) ==1) or (sim.checkCollision(center,wall30) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall31) ==1) or (sim.checkCollision(wheel_rl,wall31) ==1) or (sim.checkCollision(wheel_fr,wall31) ==1) or (sim.checkCollision(wheel_fl,wall31) ==1) or (sim.checkCollision(center,wall31) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall32) ==1) or (sim.checkCollision(wheel_rl,wall32) ==1) or (sim.checkCollision(wheel_fr,wall32) ==1) or (sim.checkCollision(wheel_fl,wall32) ==1) or (sim.checkCollision(center,wall32) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall33) ==1) or (sim.checkCollision(wheel_rl,wall33) ==1) or (sim.checkCollision(wheel_fr,wall33) ==1) or (sim.checkCollision(wheel_fl,wall33) ==1) or (sim.checkCollision(center,wall33) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall34) ==1) or (sim.checkCollision(wheel_rl,wall34) ==1) or (sim.checkCollision(wheel_fr,wall34) ==1) or (sim.checkCollision(wheel_fl,wall34) ==1) or (sim.checkCollision(center,wall34) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall35) ==1) or (sim.checkCollision(wheel_rl,wall35) ==1) or (sim.checkCollision(wheel_fr,wall35) ==1) or (sim.checkCollision(wheel_fl,wall35) ==1) or (sim.checkCollision(center,wall35) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall36) ==1) or (sim.checkCollision(wheel_rl,wall36) ==1) or (sim.checkCollision(wheel_fr,wall36) ==1) or (sim.checkCollision(wheel_fl,wall36) ==1) or (sim.checkCollision(center,wall36) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall37) ==1) or (sim.checkCollision(wheel_rl,wall37) ==1) or (sim.checkCollision(wheel_fr,wall37) ==1) or (sim.checkCollision(wheel_fl,wall37) ==1) or (sim.checkCollision(center,wall37) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall38) ==1) or (sim.checkCollision(wheel_rl,wall38) ==1) or (sim.checkCollision(wheel_fr,wall38) ==1) or (sim.checkCollision(wheel_fl,wall38) ==1) or (sim.checkCollision(center,wall38) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall39) ==1) or (sim.checkCollision(wheel_rl,wall39) ==1) or (sim.checkCollision(wheel_fr,wall39) ==1) or (sim.checkCollision(wheel_fl,wall39) ==1) or (sim.checkCollision(center,wall39) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall40) ==1) or (sim.checkCollision(wheel_rl,wall40) ==1) or (sim.checkCollision(wheel_fr,wall40) ==1) or (sim.checkCollision(wheel_fl,wall40) ==1) or (sim.checkCollision(center,wall40) ==1)) then
      colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall41) ==1) or (sim.checkCollision(wheel_rl,wall41) ==1) or (sim.checkCollision(wheel_fr,wall41) ==1) or (sim.checkCollision(wheel_fl,wall41) ==1) or (sim.checkCollision(center,wall41) ==1)) then
     colls = 1
    elseif ((sim.checkCollision(wheel_rr,wall42) ==1) or (sim.checkCollision(wheel_rl,wall42) ==1) or (sim.checkCollision(wheel_fr,wall42) ==1) or (sim.checkCollision(wheel_fl,wall42) ==1) or (sim.checkCollision(center,wall42) ==1)) then
     colls = 1
    end

    local deletwheels = 'saved wheels'
    if inInts[2]==1 then
        sim.removeObject(wheel_rr)
        sim.removeObject(wheel_rl)
        sim.removeObject(wheel_fr)
        sim.removeObject(wheel_fl)
        sim.removeObject(center)
        deletwheels = 'deleted wheels'
    end

    walpos = sim.getObjectPosition(wall42, -1)
    local colcheck = sim.createDummy(0.1)
    errorReportMode=sim.getInt32Parameter(sim.intparam_error_report_mode)
    sim.setInt32Parameter(sim.intparam_error_report_mode,0) -- temporarily suppress error output (because we are not allowed to have two times the same object name)
    sim.setObjectName(colcheck,'coll_check')
    sim.setInt32Parameter(sim.intparam_error_report_mode,errorReportMode) -- restore the original error report mode
    sim.setObjectPosition(colcheck, -1, walpos)
    colworks = sim.checkCollision(colcheck,wall42)

    return {colls, colworks, center, wheel_rr, wheel_rl, wheel_fl, wheel_fr},walpos,{deletwheels},'outbuffer' -- return a string
end


displayText_function=function(inInts,inFloats,inStrings,inBuffer)
    -- Simply display a dialog box that prints the text stored in inStrings[1]:
    if #inStrings>=1 then
        sim.displayDialog('Message from the remote API client',inStrings[1],sim.dlgstyle_ok,false)
        return {},{},{'message was displayed'},'' -- return a string
    end
end

createDummy_function=function(inInts,inFloats,inStrings,inBuffer)
    -- Create a dummy object with specific name and coordinates
    if #inStrings>=1 and #inFloats>=3 then
        local dummyHandle=sim.createDummy(0.1)
        local position={inInts[2],inInts[3],inInts[4]}
        local errorReportMode=sim.getInt32Parameter(sim.intparam_error_report_mode)
        sim.setInt32Parameter(sim.intparam_error_report_mode,0) -- temporarily suppress error output (because we are not allowed to have two times the same object name)
        sim.setObjectName(dummyHandle,inStrings[1])
        sim.setInt32Parameter(sim.intparam_error_report_mode,errorReportMode) -- restore the original error report mode
        sim.setObjectPosition(dummyHandle,-1,inFloats)
        return {dummyHandle},{},{},'' -- return the handle of the created dummy
    end
end

executeCode_function=function(inInts,inFloats,inStrings,inBuffer)
    -- Execute the code stored in inStrings[1]:
    if #inStrings>=1 then
        return {},{},{loadstring(inStrings[1])()},'' -- return a string that contains the return value of the code execution
    end
end

collision_run=function(inInts,inFloats,inStrings,inBuffer)
    -- Simply display a dialog box that prints the text stored in inStrings[1]:
    return {0},{0.0},{'hello'},'outbuffer' -- return a string
end
