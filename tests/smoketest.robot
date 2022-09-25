*** Settings ***
Library             Process
Library             OperatingSystem
Library             RequestsLibrary
Library             JSONLibrary
Library             SeleniumLibrary
Library             String

Suite Setup    Run Keywords    Check For Microros Agent    Run Qemu
Suite Teardown      Cleanup

*** Test Cases ***
Config Valid
    ${config}=    Load Json From File    ${CURDIR}/../data/config.json
    Set Global Variable    ${config}
    Validate Json By Schema File    ${config}    ${CURDIR}/../data/config-schema.json

Web Interface Available
    Open Browser    http://localhost:7654    headlesschrome
    Wait Until Element Contains    id:luaScriptEditor    function    30s
    Sleep    10

Web Interface Position Display Update
    ${target_pos}=    Convert String To JSON    [10, 20, 30, 40]
    Set Position ROS    ${target_pos}
    Check Position Web Interface    ${target_pos}
    Check Position Rest    ${target_pos}

    ${target_pos}=    Convert String To JSON    [15, 25, -35, 45]
    Set Position Rest    ${target_pos}
    Check Position Web Interface    ${target_pos}
    Check Position Rest    ${target_pos}

Web Interface Home Button
    Click Element    id:btn-home
    ${pos}=    Get Value From Json    ${config}    $.home[*]
    Check Position Web Interface    ${pos}
    Check Position Rest    ${pos}

Web Interface Manual Move Buttons
    ${pos}=    Get Value From Json    ${config}    $.limits[*].min
    Set Position Rest    ${pos}
    Check Position Rest    ${pos}
    FOR    ${axis}    IN RANGE    1    5
        Click Element    id:btn-a${axis}-fast-inc
        ${cur}=    Get Value From Json    ${pos}    $.[${${axis} - 1}]
        ${pos}=    Update Value To Json    ${pos}    $.[${${axis} - 1}]    ${${cur[0] + 10}}
        Check Position Web Interface    ${pos}
        Check Position Rest    ${pos}

        Click Element    id:btn-a${axis}-inc
        ${cur}=    Get Value From Json    ${pos}    $.[${${axis} - 1}]
        ${pos}=    Update Value To Json    ${pos}    $.[${${axis} - 1}]    ${${cur[0] + 1}}
        Check Position Web Interface    ${pos}
        Check Position Rest    ${pos}
   END

    ${pos}=    Get Value From Json    ${config}    $.limits[*].max
    Set Position Rest    ${pos}
    Check Position Rest    ${pos}
    FOR    ${axis}    IN RANGE    1    5
        Click Element    id:btn-a${axis}-fast-dec
        ${cur}=    Get Value From Json    ${pos}    $.[${${axis} - 1}]
        ${pos}=    Update Value To Json    ${pos}    $.[${${axis} - 1}]    ${${cur[0] - 10}}
        Check Position Web Interface    ${pos}
        Check Position Rest    ${pos}

        Click Element    id:btn-a${axis}-dec
        ${cur}=    Get Value From Json    ${pos}    $.[${${axis} - 1}]
        ${pos}=    Update Value To Json    ${pos}    $.[${${axis} - 1}]    ${${cur[0] - 1}}
        Check Position Web Interface    ${pos}
        Check Position Rest    ${pos}
   END

Web Interface Robo Script
    Click Element    id:btn-clear-lua
    ${script}=    Get text    id:luaScriptEditor
    Should Be Equal    ${script}    1
    Click Element    id:btn-reload-lua
    Wait Until Element Contains    id:luaScriptEditor    function    10s
    ${script}=    Get text    id:luaScriptEditor
    ${script}=    Remove String Using Regexp    ${script}    ^[0-9\n]*
    ${expect}=    Get File    ${CURDIR}/../data/script.lua
#    Should Be Equal    ${expect}    ${script}


*** Keywords ***
Run Qemu
    Log To Console    Starting Qemu...
    ${process}=    Start Process    ./qemu_run.sh    stdout=${TEMPDIR}/qemu_stdout.txt    stderr=STDOUT
    FOR    ${counter}    IN RANGE    1   120
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
    FOR     ${axis}     IN RANGE     1    5
        ${pos}=    Get Value From Json    ${expected_pos}    $.[${${axis}-1}]
        Wait Until Element Contains    id:pos-a${axis}    ${pos[0]}
    END

Set Position Rest
    [Arguments]    ${pos}
    ${str_pos}    Convert JSON To String    ${pos}
    ${resp}=    POST    http://localhost:7654/rest/set_joint_angles    ${str_pos}
    Status Should Be    OK    ${resp}

Set Position ROS
    [Arguments]    ${pos}
    ${result}=    Run Process    ./send_ros_pos.sh    ${pos}
    Should Be Empty    ${result.stderr}
