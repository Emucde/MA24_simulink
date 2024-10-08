    function targMap = targDataMap(),

    ;%***********************
    ;% Create Parameter Map *
    ;%***********************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 1;
        sectIdxOffset = 0;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc paramMap
        ;%
        paramMap.nSections           = nTotSects;
        paramMap.sectIdxOffset       = sectIdxOffset;
            paramMap.sections(nTotSects) = dumSection; %prealloc
        paramMap.nTotData            = -1;

        ;%
        ;% Auto data (cartesian_impedance_control_P)
        ;%
            section.nData     = 15;
            section.data(15)  = dumData; %prealloc

                    ;% cartesian_impedance_control_P.q_init
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% cartesian_impedance_control_P.Switch_Threshold
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 7;

                    ;% cartesian_impedance_control_P.Constant8_Value
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 8;

                    ;% cartesian_impedance_control_P.Constant9_Value
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 15;

                    ;% cartesian_impedance_control_P.q_d_3_Value
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 22;

                    ;% cartesian_impedance_control_P.RateLimiter_RisingLim
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 29;

                    ;% cartesian_impedance_control_P.RateLimiter_FallingLim
                    section.data(7).logicalSrcIdx = 6;
                    section.data(7).dtTransOffset = 30;

                    ;% cartesian_impedance_control_P.RateLimiter_IC
                    section.data(8).logicalSrcIdx = 7;
                    section.data(8).dtTransOffset = 31;

                    ;% cartesian_impedance_control_P.ApplyControl_P1
                    section.data(9).logicalSrcIdx = 8;
                    section.data(9).dtTransOffset = 32;

                    ;% cartesian_impedance_control_P.ApplyControl_P2
                    section.data(10).logicalSrcIdx = 9;
                    section.data(10).dtTransOffset = 84;

                    ;% cartesian_impedance_control_P.ApplyControl_P3
                    section.data(11).logicalSrcIdx = 10;
                    section.data(11).dtTransOffset = 91;

                    ;% cartesian_impedance_control_P.ApplyControl_P4
                    section.data(12).logicalSrcIdx = 11;
                    section.data(12).dtTransOffset = 97;

                    ;% cartesian_impedance_control_P.ApplyControl_P5
                    section.data(13).logicalSrcIdx = 12;
                    section.data(13).dtTransOffset = 110;

                    ;% cartesian_impedance_control_P.ApplyControl_P6
                    section.data(14).logicalSrcIdx = 13;
                    section.data(14).dtTransOffset = 126;

                    ;% cartesian_impedance_control_P.Constant10_Value
                    section.data(15).logicalSrcIdx = 14;
                    section.data(15).dtTransOffset = 133;

            nTotData = nTotData + section.nData;
            paramMap.sections(1) = section;
            clear section


            ;%
            ;% Non-auto Data (parameter)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        paramMap.nTotData = nTotData;



    ;%**************************
    ;% Create Block Output Map *
    ;%**************************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 1;
        sectIdxOffset = 0;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc sigMap
        ;%
        sigMap.nSections           = nTotSects;
        sigMap.sectIdxOffset       = sectIdxOffset;
            sigMap.sections(nTotSects) = dumSection; %prealloc
        sigMap.nTotData            = -1;

        ;%
        ;% Auto data (cartesian_impedance_control_B)
        ;%
            section.nData     = 8;
            section.data(8)  = dumData; %prealloc

                    ;% cartesian_impedance_control_B.GetRobotState2_o1
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% cartesian_impedance_control_B.GetRobotState2_o2
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 7;

                    ;% cartesian_impedance_control_B.GetRobotState2_o3
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 14;

                    ;% cartesian_impedance_control_B.Switch
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 20;

                    ;% cartesian_impedance_control_B.RateLimiter
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 27;

                    ;% cartesian_impedance_control_B.data_out
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 34;

                    ;% cartesian_impedance_control_B.bytes
                    section.data(7).logicalSrcIdx = 6;
                    section.data(7).dtTransOffset = 35;

                    ;% cartesian_impedance_control_B.missed_data_cnt_o
                    section.data(8).logicalSrcIdx = 7;
                    section.data(8).dtTransOffset = 36;

            nTotData = nTotData + section.nData;
            sigMap.sections(1) = section;
            clear section


            ;%
            ;% Non-auto Data (signal)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        sigMap.nTotData = nTotData;



    ;%*******************
    ;% Create DWork Map *
    ;%*******************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 4;
        sectIdxOffset = 1;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc dworkMap
        ;%
        dworkMap.nSections           = nTotSects;
        dworkMap.sectIdxOffset       = sectIdxOffset;
            dworkMap.sections(nTotSects) = dumSection; %prealloc
        dworkMap.nTotData            = -1;

        ;%
        ;% Auto data (cartesian_impedance_control_DW)
        ;%
            section.nData     = 6;
            section.data(6)  = dumData; %prealloc

                    ;% cartesian_impedance_control_DW.GetRobotState2_DWORK1
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% cartesian_impedance_control_DW.PrevY
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 1;

                    ;% cartesian_impedance_control_DW.ApplyControl_DWORK1
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 8;

                    ;% cartesian_impedance_control_DW.ApplyControl_DWORK2
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 9;

                    ;% cartesian_impedance_control_DW.data_prev
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 10;

                    ;% cartesian_impedance_control_DW.missed_data_cnt
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 11;

            nTotData = nTotData + section.nData;
            dworkMap.sections(1) = section;
            clear section

            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% cartesian_impedance_control_DW.Scope_PWORK.LoggedData
                    section.data(1).logicalSrcIdx = 6;
                    section.data(1).dtTransOffset = 0;

                    ;% cartesian_impedance_control_DW.Scope6_PWORK.LoggedData
                    section.data(2).logicalSrcIdx = 7;
                    section.data(2).dtTransOffset = 3;

            nTotData = nTotData + section.nData;
            dworkMap.sections(2) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% cartesian_impedance_control_DW.eml_autoflush
                    section.data(1).logicalSrcIdx = 8;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(3) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% cartesian_impedance_control_DW.eml_openfiles
                    section.data(1).logicalSrcIdx = 9;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(4) = section;
            clear section


            ;%
            ;% Non-auto Data (dwork)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        dworkMap.nTotData = nTotData;



    ;%
    ;% Add individual maps to base struct.
    ;%

    targMap.paramMap  = paramMap;
    targMap.signalMap = sigMap;
    targMap.dworkMap  = dworkMap;

    ;%
    ;% Add checksums to base struct.
    ;%


    targMap.checksum0 = 3189320934;
    targMap.checksum1 = 2339537802;
    targMap.checksum2 = 4272940836;
    targMap.checksum3 = 1849617939;

