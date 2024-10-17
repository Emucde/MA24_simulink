    function targMap = targDataMap(),

    ;%***********************
    ;% Create Parameter Map *
    ;%***********************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 3;
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
            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% cartesian_impedance_control_P.ctrl_param
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            paramMap.sections(1) = section;
            clear section

            section.nData     = 27;
            section.data(27)  = dumData; %prealloc

                    ;% cartesian_impedance_control_P.q_init
                    section.data(1).logicalSrcIdx = 1;
                    section.data(1).dtTransOffset = 0;

                    ;% cartesian_impedance_control_P.SFunction3_P1_Size
                    section.data(2).logicalSrcIdx = 2;
                    section.data(2).dtTransOffset = 7;

                    ;% cartesian_impedance_control_P.SFunction3_P1
                    section.data(3).logicalSrcIdx = 3;
                    section.data(3).dtTransOffset = 9;

                    ;% cartesian_impedance_control_P.SFunction3_P2_Size
                    section.data(4).logicalSrcIdx = 4;
                    section.data(4).dtTransOffset = 25;

                    ;% cartesian_impedance_control_P.SFunction3_P2
                    section.data(5).logicalSrcIdx = 5;
                    section.data(5).dtTransOffset = 27;

                    ;% cartesian_impedance_control_P.Switch_Threshold
                    section.data(6).logicalSrcIdx = 6;
                    section.data(6).dtTransOffset = 49;

                    ;% cartesian_impedance_control_P.SFunction4_P1_Size
                    section.data(7).logicalSrcIdx = 7;
                    section.data(7).dtTransOffset = 50;

                    ;% cartesian_impedance_control_P.SFunction4_P1
                    section.data(8).logicalSrcIdx = 8;
                    section.data(8).dtTransOffset = 52;

                    ;% cartesian_impedance_control_P.SFunction4_P2_Size
                    section.data(9).logicalSrcIdx = 9;
                    section.data(9).dtTransOffset = 70;

                    ;% cartesian_impedance_control_P.SFunction4_P2
                    section.data(10).logicalSrcIdx = 10;
                    section.data(10).dtTransOffset = 72;

                    ;% cartesian_impedance_control_P.tau_in_Value
                    section.data(11).logicalSrcIdx = 11;
                    section.data(11).dtTransOffset = 96;

                    ;% cartesian_impedance_control_P.cartesiandamping_Value
                    section.data(12).logicalSrcIdx = 12;
                    section.data(12).dtTransOffset = 103;

                    ;% cartesian_impedance_control_P.cartesianstiffness_Value
                    section.data(13).logicalSrcIdx = 13;
                    section.data(13).dtTransOffset = 139;

                    ;% cartesian_impedance_control_P.Constant5_Value
                    section.data(14).logicalSrcIdx = 14;
                    section.data(14).dtTransOffset = 175;

                    ;% cartesian_impedance_control_P.Constant4_Value
                    section.data(15).logicalSrcIdx = 15;
                    section.data(15).dtTransOffset = 176;

                    ;% cartesian_impedance_control_P.Constant9_Value
                    section.data(16).logicalSrcIdx = 16;
                    section.data(16).dtTransOffset = 177;

                    ;% cartesian_impedance_control_P.Constant8_Value
                    section.data(17).logicalSrcIdx = 17;
                    section.data(17).dtTransOffset = 184;

                    ;% cartesian_impedance_control_P.q_d_3_Value
                    section.data(18).logicalSrcIdx = 18;
                    section.data(18).dtTransOffset = 191;

                    ;% cartesian_impedance_control_P.RateLimiter_RisingLim
                    section.data(19).logicalSrcIdx = 19;
                    section.data(19).dtTransOffset = 198;

                    ;% cartesian_impedance_control_P.RateLimiter_FallingLim
                    section.data(20).logicalSrcIdx = 20;
                    section.data(20).dtTransOffset = 199;

                    ;% cartesian_impedance_control_P.RateLimiter_IC
                    section.data(21).logicalSrcIdx = 21;
                    section.data(21).dtTransOffset = 200;

                    ;% cartesian_impedance_control_P.ApplyControl_P1
                    section.data(22).logicalSrcIdx = 22;
                    section.data(22).dtTransOffset = 201;

                    ;% cartesian_impedance_control_P.ApplyControl_P2
                    section.data(23).logicalSrcIdx = 23;
                    section.data(23).dtTransOffset = 253;

                    ;% cartesian_impedance_control_P.ApplyControl_P3
                    section.data(24).logicalSrcIdx = 24;
                    section.data(24).dtTransOffset = 260;

                    ;% cartesian_impedance_control_P.ApplyControl_P4
                    section.data(25).logicalSrcIdx = 25;
                    section.data(25).dtTransOffset = 266;

                    ;% cartesian_impedance_control_P.ApplyControl_P5
                    section.data(26).logicalSrcIdx = 26;
                    section.data(26).dtTransOffset = 279;

                    ;% cartesian_impedance_control_P.ApplyControl_P6
                    section.data(27).logicalSrcIdx = 27;
                    section.data(27).dtTransOffset = 295;

            nTotData = nTotData + section.nData;
            paramMap.sections(2) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% cartesian_impedance_control_P.ManualSwitch_CurrentSetting
                    section.data(1).logicalSrcIdx = 28;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            paramMap.sections(3) = section;
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
            section.nData     = 24;
            section.data(24)  = dumData; %prealloc

                    ;% cartesian_impedance_control_B.GetRobotState2_o1
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% cartesian_impedance_control_B.GetRobotState2_o2
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 7;

                    ;% cartesian_impedance_control_B.GetRobotState2_o3
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 14;

                    ;% cartesian_impedance_control_B.SFunction3_o1
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 20;

                    ;% cartesian_impedance_control_B.SFunction3_o2
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 270;

                    ;% cartesian_impedance_control_B.Switch
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 271;

                    ;% cartesian_impedance_control_B.ManualSwitch
                    section.data(7).logicalSrcIdx = 6;
                    section.data(7).dtTransOffset = 278;

                    ;% cartesian_impedance_control_B.tau_in
                    section.data(8).logicalSrcIdx = 7;
                    section.data(8).dtTransOffset = 292;

                    ;% cartesian_impedance_control_B.SFunction1
                    section.data(9).logicalSrcIdx = 8;
                    section.data(9).dtTransOffset = 299;

                    ;% cartesian_impedance_control_B.H
                    section.data(10).logicalSrcIdx = 9;
                    section.data(10).dtTransOffset = 306;

                    ;% cartesian_impedance_control_B.J
                    section.data(11).logicalSrcIdx = 10;
                    section.data(11).dtTransOffset = 322;

                    ;% cartesian_impedance_control_B.J_p
                    section.data(12).logicalSrcIdx = 11;
                    section.data(12).dtTransOffset = 364;

                    ;% cartesian_impedance_control_B.M
                    section.data(13).logicalSrcIdx = 12;
                    section.data(13).dtTransOffset = 406;

                    ;% cartesian_impedance_control_B.Cqg
                    section.data(14).logicalSrcIdx = 13;
                    section.data(14).dtTransOffset = 455;

                    ;% cartesian_impedance_control_B.g
                    section.data(15).logicalSrcIdx = 14;
                    section.data(15).dtTransOffset = 462;

                    ;% cartesian_impedance_control_B.g_f
                    section.data(16).logicalSrcIdx = 15;
                    section.data(16).dtTransOffset = 511;

                    ;% cartesian_impedance_control_B.RateLimiter
                    section.data(17).logicalSrcIdx = 16;
                    section.data(17).dtTransOffset = 518;

                    ;% cartesian_impedance_control_B.tau
                    section.data(18).logicalSrcIdx = 17;
                    section.data(18).dtTransOffset = 525;

                    ;% cartesian_impedance_control_B.p_d
                    section.data(19).logicalSrcIdx = 18;
                    section.data(19).dtTransOffset = 532;

                    ;% cartesian_impedance_control_B.tau_h
                    section.data(20).logicalSrcIdx = 19;
                    section.data(20).dtTransOffset = 535;

                    ;% cartesian_impedance_control_B.y
                    section.data(21).logicalSrcIdx = 20;
                    section.data(21).dtTransOffset = 537;

                    ;% cartesian_impedance_control_B.simulink_valid_flag
                    section.data(22).logicalSrcIdx = 21;
                    section.data(22).dtTransOffset = 551;

                    ;% cartesian_impedance_control_B.p
                    section.data(23).logicalSrcIdx = 22;
                    section.data(23).dtTransOffset = 552;

                    ;% cartesian_impedance_control_B.tau_c
                    section.data(24).logicalSrcIdx = 23;
                    section.data(24).dtTransOffset = 555;

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
        nTotSects     = 2;
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
            section.nData     = 5;
            section.data(5)  = dumData; %prealloc

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

                    ;% cartesian_impedance_control_DW.cnt
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 10;

            nTotData = nTotData + section.nData;
            dworkMap.sections(1) = section;
            clear section

            section.nData     = 8;
            section.data(8)  = dumData; %prealloc

                    ;% cartesian_impedance_control_DW.SFunction3_PWORK
                    section.data(1).logicalSrcIdx = 5;
                    section.data(1).dtTransOffset = 0;

                    ;% cartesian_impedance_control_DW.SFunction4_PWORK
                    section.data(2).logicalSrcIdx = 6;
                    section.data(2).dtTransOffset = 4;

                    ;% cartesian_impedance_control_DW.SFunction1_PWORK
                    section.data(3).logicalSrcIdx = 7;
                    section.data(3).dtTransOffset = 8;

                    ;% cartesian_impedance_control_DW.SFunction2_PWORK
                    section.data(4).logicalSrcIdx = 8;
                    section.data(4).dtTransOffset = 12;

                    ;% cartesian_impedance_control_DW.Scope5_PWORK.LoggedData
                    section.data(5).logicalSrcIdx = 9;
                    section.data(5).dtTransOffset = 21;

                    ;% cartesian_impedance_control_DW.Scope_PWORK.LoggedData
                    section.data(6).logicalSrcIdx = 10;
                    section.data(6).dtTransOffset = 24;

                    ;% cartesian_impedance_control_DW.errscopeall_PWORK.LoggedData
                    section.data(7).logicalSrcIdx = 11;
                    section.data(7).dtTransOffset = 27;

                    ;% cartesian_impedance_control_DW.Scope6_PWORK.LoggedData
                    section.data(8).logicalSrcIdx = 12;
                    section.data(8).dtTransOffset = 28;

            nTotData = nTotData + section.nData;
            dworkMap.sections(2) = section;
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


    targMap.checksum0 = 4056965307;
    targMap.checksum1 = 428666425;
    targMap.checksum2 = 449071374;
    targMap.checksum3 = 2890886619;

