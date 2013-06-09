<?xml version="1.0" ?>
<dart version="1.0">
    <world name="world 1">
        <physics>
            <time_step>0.001</time_step>
            <gravity>0 -9.81 0</gravity>
        </physics>
        <skeleton name="grount skeleton">
            <body name="ground">
                <transformation>0 0 0 0 -0.5 0</transformation>
                <visualization_shape>
                    <transformation>0 0 0 0 0 0</transformation>
                    <geometry>
                        <box>
                            <size>2.0 0.01 2.0</size>
                        </box>
                    </geometry>
                </visualization_shape>
                <collision_shape>
                    <transformation>0 0 0 0 0 0</transformation>
                    <geometry>
                        <box>
                            <size>2.0 0.01 2.0</size>
                        </box>
                    </geometry>
                </collision_shape>                                
            </body>
            <joint type="weld" name="joint 1">
                <parent>world</parent>
                <child>ground</child>
            </joint>
        </skeleton>	
        
        <skeleton name="box skeleton">
            <body name="box">
                <gravity>1</gravity>
                <transformation>1 2 3 0 0.5 0</transformation>
                <inertia>
                    <mass>1</mass>
                    <offset>0 0 0</offset>
                    <moment_of_inertia>
                        <ixx>0.01</ixx>
                        <iyy>0.01</iyy>
                        <izz>0.01</izz>
                        <ixy>0</ixy>
                        <ixz>0</ixz>
                        <iyz>0</iyz>
                    </moment_of_inertia>
                </inertia>
                <visualization_shape>
                    <transformation>0 0 0 0 0 0</transformation>
                    <geometry>
                        <box>
                            <size>0.2 0.2 0.2</size>
                        </box>
                    </geometry>
                </visualization_shape>
                <collision_shape>
                    <transformation>0 0 0 0 0 0</transformation>
                    <geometry>
                        <box>
                            <size>0.2 0.2 0.2</size>
                        </box>
                    </geometry>
                </collision_shape>                                
            </body>
            
            <joint type="free" name="joint 1">
                <parent>world</parent>
                <child>box</child>
            </joint>
        </skeleton>	
        
    </world>
</dart>