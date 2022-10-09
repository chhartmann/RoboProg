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
    Set Window Size    800    1000
    Sleep    10

Web Interface Position Display Update
    ${pos}=    Convert String To JSON    [10, 20, 30, 40]
    Set Position ROS    ${pos}
    Check Position Web Interface    ${pos}
    Check Position Rest    ${pos}

    ${pos}=    Convert String To JSON    [15, 25, -35, 45]
    Set Position Rest    ${pos}
    Check Position Web Interface    ${pos}
    Check Position Rest    ${pos}

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

Web Interface Robo Script Reload And Clear
    Click Element    id:btn-clear-lua
    ${script}=    Get text    id:luaScriptEditor
    Should Be Equal    ${script}    1
    Click Element    id:btn-reload-lua
    Wait Until Element Contains    id:luaScriptEditor    function    10s
    ${visible_script}=    Get text    css:div.ace_content
    ${full_script}=    Get File    ${CURDIR}/../data/script.lua
    Should Contain    ${{$full_script.replace('\n', '')}}    ${{$visible_script.replace('\n', '')}}

Web Interface Robo Script MoveTo And Clear
    ${pos}=    Convert String To JSON    [10, 20, 30, 40]
    Set Position Rest    ${pos}
    Check Position Web Interface    ${pos}
    Click Element    id:btn-clear-lua
    Wait Until Element Does Not Contain    css:div.ace_content    function    10s
    Click Element    id:btn-touchup-lua
    Wait Until Element Contains    css:div.ace_content    setJointAngles(10,20,30,40)

Web Interface Robo Script Save And Reload
    Set Local Variable    ${script}    logWeb("hello world")
    Execute Javascript    window.luaEditor.setValue('${script}')
    Wait Until Element Contains    css:div.ace_content    ${script}    10s
    Click Element    id:btn-save-lua
    Click Element    id:btn-clear-lua
    Wait Until Element Does Not Contain    css:div.ace_content    ${script}    10s
    Click Element    id:btn-reload-lua
    Wait Until Element Contains    css:div.ace_content    ${script}    30s
    ${restore}=    Get File    ${CURDIR}/../data/script.lua
    POST    http://localhost:7654/upload/script.lua    ${restore}

Web Interface Robo Script Run
    Click Element    id:btn-run-lua
    Wait Until Web Log Contains    Lua task started
    Wait Until Web Log Contains    hello world
    Wait Until Web Log Contains    Lua task finished
    Wait Until Web Log Contains    ready

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

Wait Until Web Log Contains
    [Arguments]    ${part}
    FOR    ${counter}    IN RANGE    1   10
    Sleep    1s
    ${text}=    Get Element Attribute   id:luaScriptOutput    value
        IF    "${part}" in """${text}"""
            RETURN
        END
    END
    ${text}=    Get Element Attribute   id:luaScriptOutput    value
    Fail    "${pos}"" not in "${text}"
