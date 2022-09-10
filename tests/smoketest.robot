*** Settings ***
Library             Process
Library             OperatingSystem
Library             RequestsLibrary
Library             JSONLibrary

Suite Setup    Run Qemu
Suite Teardown      Terminate All Processes    kill=True


*** Test Cases ***
Rest Api
    ${pos}=    Get Position Rest
    ${len}=    Get Length    ${pos}
    Should Be Equal As Integers    ${len}    4

*** Keywords ***
Run Qemu
    Log To Console    Starting Qemu...
    Start Process    ./build_qemu.sh    stdout=${TEMPDIR}/stdout.txt
    FOR    ${counter}    IN RANGE    1    60
        Sleep    1s
        ${processStatus}=    Is Process Running
        IF    ${processStatus}
            ${stdout}=    Get File    ${TEMPDIR}/stdout.txt
            IF    "rpg: Setup finished" in """${stdout}"""
                Log To Console    Qemu is up with RoboProg running"
                RETURN
            END
        ELSE
            BREAK
        END
    END
    Log    ${stdout}
    Fail    "Start Qemu failed"

Get Position Rest
    ${response}=    GET    http://localhost:7654/rest/get_joint_angles
    Should Be Equal As Strings    ${response.reason}    OK
    ${result}=    Convert String to JSON    ${response.text}
    [Return]    ${result}
