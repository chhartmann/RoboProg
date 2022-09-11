*** Settings ***
Library             Process
Library             OperatingSystem
Library             RequestsLibrary
Library             JSONLibrary

Suite Setup    Run Keywords    Run Microros Agent    Run Qemu
Suite Teardown      Stop Processes


*** Test Cases ***
Rest Api
    ${pos}=    Get Position Rest
    ${len}=    Get Length    ${pos}
    Should Be Equal As Integers    ${len}    4

ROS Interface
    ${target_pos}=    Convert String To JSON    [10, 20, 30, 40]
    ${result}=    Run Process    ros2    topic    pub    --once    /micro_ros_robo_prog_subscriber    std_msgs/msg/Int32MultiArray    {'data': ${target_pos}}
    Should Be Empty    ${result.stderr}
    ${pos}=    Get Position Rest
    Should Be Equal    ${pos}    ${target_pos}

*** Keywords ***
Run Qemu
    Log To Console    Starting Qemu...
    Start Process    ./build_qemu.sh    stdout=${TEMPDIR}/stdout.txt
    FOR    ${counter}    IN RANGE    1    90
        Sleep    1s
        ${processStatus}=    Is Process Running
        IF    ${processStatus}
            ${stdout}=    Get File    ${TEMPDIR}/stdout.txt
            IF    "rpg: Setup finished" in """${stdout}"""
                Log To Console    Qemu is up with RoboProg running"
                RETURN
            END
        ELSE
            Log    ${stdout}
            BREAK
        END
    END
    Fail    "Start Qemu failed"

Run Microros Agent
    Log To Console    Starting Microros Agent
    ${result}=    Run Process    ./start_microros_agent.sh
    Should Be Empty    ${result.stderr}

Stop Processes
    Terminate All Processes    kill=True
    Run Process    ./stop_microros_agent.sh

Get Position Rest
    ${response}=    GET    http://localhost:7654/rest/get_joint_angles
    Should Be Equal As Strings    ${response.reason}    OK
    ${result}=    Convert String to JSON    ${response.text}
    RETURN    ${result}
