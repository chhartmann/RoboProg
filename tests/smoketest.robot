*** Settings ***
Library             Process
Library             OperatingSystem
Library             RequestsLibrary
Library             JSONLibrary
Library             SeleniumLibrary

Suite Setup    Run Keywords    Check For Microros Agent    Run Qemu
Suite Teardown      Cleanup

*** Test Cases ***
Web Interface
    Open Browser    http://localhost:7654    headlesschrome
    Page Should Contain    function

    ${config}=    Load Json From File    ${CURDIR}/../data/config.json
    Validate Json By Schema File    ${config}    ${CURDIR}/../data/config-schema.json

    # Click Button    Home
    # ${pos}=    Get Value From Json    ${config}    $.home
    # Check Position Rest    ${config}
    # Check Position Web Interface    ${config}

    ${target_pos}=    Convert String To JSON    [10, 20, 30, 40]
    Set Position ROS    ${target_pos}
    Check Position Rest    ${target_pos}
    Check Position Web Interface    ${target_pos}

    ${target_pos}=    Convert String To JSON    [15, 25, -35, 45]
    Set Position Rest    ${target_pos}
    Check Position Rest    ${target_pos}
    Check Position Web Interface    ${target_pos}

*** Keywords ***
Run Qemu
    Log To Console    Starting Qemu...
    ${process}=    Start Process    ./qemu_run.sh    stdout=${TEMPDIR}/qemu_stdout.txt    stderr=STDOUT
    FOR    ${counter}    IN RANGE    1   90
        Sleep    1s
        ${processStatus}=    Is Process Running
        IF    ${processStatus}
            ${stdout}=    Get File    ${TEMPDIR}/qemu_stdout.txt
            IF    "rpg: Setup finished" in """${stdout}"""
                Log To Console    Qemu is up with RoboProg running"
                RETURN
            END
        ELSE
            BREAK
        END
    END
    ${stdout}=    Get File    ${TEMPDIR}/qemu_stdout.txt
    Log    ${stdout}
    Log To Console    ${stdout}
    Fail    "Start Qemu failed"

Cleanup
    Terminate All Processes    kill=True
    Run Process    pkill    -f    Chrome
    Run Process    pkill    -f    chromedriver

Check For Microros Agent
    ${process}=    Run Process    netstat    -an    udp4
    Should Contain    ${process.stdout}    :8888

Check Position Rest
    [Arguments]    ${expected_pos}
    ${response}=    GET    http://localhost:7654/rest/get_joint_angles
    Should Be Equal As Strings    ${response.reason}    OK
    ${result}=    Convert String to JSON    ${response.text}
    Should Be Equal    ${expected_pos}    ${result}

Check Position Web Interface
    [Arguments]    ${expected_pos}
    Sleep    2s
    ${a1}=    Get Text    id:pos-a1
    ${a2}=    Get Text    id:pos-a2
    ${a3}=    Get Text    id:pos-a3
    ${a4}=    Get Text    id:pos-a4
    ${result}=    Convert String to JSON    [${a1}, ${a2}, ${a3}, ${a4}]
    Should Be Equal    ${expected_pos}    ${result}

Set Position Rest
    [Arguments]    ${pos}
    ${str_pos}    Convert JSON To String    ${pos}
    ${resp}=    POST    http://localhost:7654/rest/set_joint_angles    ${str_pos}
    Status Should Be    OK    ${resp}

Set Position ROS
    [Arguments]    ${pos}
    ${result}=    Run Process    ./send_ros_pos.sh    ${pos}
    Should Be Empty    ${result.stderr}

