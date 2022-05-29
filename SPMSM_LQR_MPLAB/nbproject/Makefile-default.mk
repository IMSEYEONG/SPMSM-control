#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/Lab2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/Lab2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS
SUB_IMAGE_ADDRESS_COMMAND=--image-address $(SUB_IMAGE_ADDRESS)
else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=atan2CORDIC.s CalcRef.s clrkpark.s FdWeak.c InvPark.s MeasCurr.s pi.s ReadADC0.s RTDM.c Lab2.c smc.s smcpos.c SVGEN.S trig.s initdspic.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/atan2CORDIC.o ${OBJECTDIR}/CalcRef.o ${OBJECTDIR}/clrkpark.o ${OBJECTDIR}/FdWeak.o ${OBJECTDIR}/InvPark.o ${OBJECTDIR}/MeasCurr.o ${OBJECTDIR}/pi.o ${OBJECTDIR}/ReadADC0.o ${OBJECTDIR}/RTDM.o ${OBJECTDIR}/Lab2.o ${OBJECTDIR}/smc.o ${OBJECTDIR}/smcpos.o ${OBJECTDIR}/SVGEN.o ${OBJECTDIR}/trig.o ${OBJECTDIR}/initdspic.o
POSSIBLE_DEPFILES=${OBJECTDIR}/atan2CORDIC.o.d ${OBJECTDIR}/CalcRef.o.d ${OBJECTDIR}/clrkpark.o.d ${OBJECTDIR}/FdWeak.o.d ${OBJECTDIR}/InvPark.o.d ${OBJECTDIR}/MeasCurr.o.d ${OBJECTDIR}/pi.o.d ${OBJECTDIR}/ReadADC0.o.d ${OBJECTDIR}/RTDM.o.d ${OBJECTDIR}/Lab2.o.d ${OBJECTDIR}/smc.o.d ${OBJECTDIR}/smcpos.o.d ${OBJECTDIR}/SVGEN.o.d ${OBJECTDIR}/trig.o.d ${OBJECTDIR}/initdspic.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/atan2CORDIC.o ${OBJECTDIR}/CalcRef.o ${OBJECTDIR}/clrkpark.o ${OBJECTDIR}/FdWeak.o ${OBJECTDIR}/InvPark.o ${OBJECTDIR}/MeasCurr.o ${OBJECTDIR}/pi.o ${OBJECTDIR}/ReadADC0.o ${OBJECTDIR}/RTDM.o ${OBJECTDIR}/Lab2.o ${OBJECTDIR}/smc.o ${OBJECTDIR}/smcpos.o ${OBJECTDIR}/SVGEN.o ${OBJECTDIR}/trig.o ${OBJECTDIR}/initdspic.o

# Source Files
SOURCEFILES=atan2CORDIC.s CalcRef.s clrkpark.s FdWeak.c InvPark.s MeasCurr.s pi.s ReadADC0.s RTDM.c Lab2.c smc.s smcpos.c SVGEN.S trig.s initdspic.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/Lab2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=33EP256MC506
MP_LINKER_FILE_OPTION=,--script=p33EP256MC506.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/FdWeak.o: FdWeak.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/FdWeak.o.d 
	@${RM} ${OBJECTDIR}/FdWeak.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FdWeak.c  -o ${OBJECTDIR}/FdWeak.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/FdWeak.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/FdWeak.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/RTDM.o: RTDM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/RTDM.o.d 
	@${RM} ${OBJECTDIR}/RTDM.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  RTDM.c  -o ${OBJECTDIR}/RTDM.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/RTDM.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/RTDM.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/Lab2.o: Lab2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Lab2.o.d 
	@${RM} ${OBJECTDIR}/Lab2.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  Lab2.c  -o ${OBJECTDIR}/Lab2.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/Lab2.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/Lab2.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/smcpos.o: smcpos.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/smcpos.o.d 
	@${RM} ${OBJECTDIR}/smcpos.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  smcpos.c  -o ${OBJECTDIR}/smcpos.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/smcpos.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/smcpos.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/initdspic.o: initdspic.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/initdspic.o.d 
	@${RM} ${OBJECTDIR}/initdspic.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  initdspic.c  -o ${OBJECTDIR}/initdspic.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/initdspic.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/initdspic.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/FdWeak.o: FdWeak.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/FdWeak.o.d 
	@${RM} ${OBJECTDIR}/FdWeak.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FdWeak.c  -o ${OBJECTDIR}/FdWeak.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/FdWeak.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/FdWeak.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/RTDM.o: RTDM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/RTDM.o.d 
	@${RM} ${OBJECTDIR}/RTDM.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  RTDM.c  -o ${OBJECTDIR}/RTDM.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/RTDM.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/RTDM.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/Lab2.o: Lab2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Lab2.o.d 
	@${RM} ${OBJECTDIR}/Lab2.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  Lab2.c  -o ${OBJECTDIR}/Lab2.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/Lab2.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/Lab2.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/smcpos.o: smcpos.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/smcpos.o.d 
	@${RM} ${OBJECTDIR}/smcpos.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  smcpos.c  -o ${OBJECTDIR}/smcpos.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/smcpos.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/smcpos.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/initdspic.o: initdspic.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/initdspic.o.d 
	@${RM} ${OBJECTDIR}/initdspic.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  initdspic.c  -o ${OBJECTDIR}/initdspic.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/initdspic.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/initdspic.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/atan2CORDIC.o: atan2CORDIC.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/atan2CORDIC.o.d 
	@${RM} ${OBJECTDIR}/atan2CORDIC.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  atan2CORDIC.s  -o ${OBJECTDIR}/atan2CORDIC.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/atan2CORDIC.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_REAL_ICE=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/atan2CORDIC.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/CalcRef.o: CalcRef.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/CalcRef.o.d 
	@${RM} ${OBJECTDIR}/CalcRef.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  CalcRef.s  -o ${OBJECTDIR}/CalcRef.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/CalcRef.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_REAL_ICE=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/CalcRef.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/clrkpark.o: clrkpark.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/clrkpark.o.d 
	@${RM} ${OBJECTDIR}/clrkpark.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  clrkpark.s  -o ${OBJECTDIR}/clrkpark.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/clrkpark.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_REAL_ICE=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/clrkpark.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/InvPark.o: InvPark.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/InvPark.o.d 
	@${RM} ${OBJECTDIR}/InvPark.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  InvPark.s  -o ${OBJECTDIR}/InvPark.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/InvPark.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_REAL_ICE=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/InvPark.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MeasCurr.o: MeasCurr.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MeasCurr.o.d 
	@${RM} ${OBJECTDIR}/MeasCurr.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  MeasCurr.s  -o ${OBJECTDIR}/MeasCurr.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/MeasCurr.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_REAL_ICE=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/MeasCurr.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/pi.o: pi.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/pi.o.d 
	@${RM} ${OBJECTDIR}/pi.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  pi.s  -o ${OBJECTDIR}/pi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/pi.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_REAL_ICE=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/pi.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ReadADC0.o: ReadADC0.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/ReadADC0.o.d 
	@${RM} ${OBJECTDIR}/ReadADC0.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ReadADC0.s  -o ${OBJECTDIR}/ReadADC0.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/ReadADC0.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_REAL_ICE=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/ReadADC0.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/smc.o: smc.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/smc.o.d 
	@${RM} ${OBJECTDIR}/smc.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  smc.s  -o ${OBJECTDIR}/smc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/smc.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_REAL_ICE=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/smc.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/trig.o: trig.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/trig.o.d 
	@${RM} ${OBJECTDIR}/trig.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  trig.s  -o ${OBJECTDIR}/trig.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/trig.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_REAL_ICE=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/trig.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
else
${OBJECTDIR}/atan2CORDIC.o: atan2CORDIC.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/atan2CORDIC.o.d 
	@${RM} ${OBJECTDIR}/atan2CORDIC.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  atan2CORDIC.s  -o ${OBJECTDIR}/atan2CORDIC.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/atan2CORDIC.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/atan2CORDIC.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/CalcRef.o: CalcRef.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/CalcRef.o.d 
	@${RM} ${OBJECTDIR}/CalcRef.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  CalcRef.s  -o ${OBJECTDIR}/CalcRef.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/CalcRef.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/CalcRef.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/clrkpark.o: clrkpark.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/clrkpark.o.d 
	@${RM} ${OBJECTDIR}/clrkpark.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  clrkpark.s  -o ${OBJECTDIR}/clrkpark.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/clrkpark.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/clrkpark.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/InvPark.o: InvPark.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/InvPark.o.d 
	@${RM} ${OBJECTDIR}/InvPark.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  InvPark.s  -o ${OBJECTDIR}/InvPark.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/InvPark.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/InvPark.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MeasCurr.o: MeasCurr.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MeasCurr.o.d 
	@${RM} ${OBJECTDIR}/MeasCurr.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  MeasCurr.s  -o ${OBJECTDIR}/MeasCurr.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/MeasCurr.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/MeasCurr.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/pi.o: pi.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/pi.o.d 
	@${RM} ${OBJECTDIR}/pi.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  pi.s  -o ${OBJECTDIR}/pi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/pi.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/pi.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ReadADC0.o: ReadADC0.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/ReadADC0.o.d 
	@${RM} ${OBJECTDIR}/ReadADC0.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ReadADC0.s  -o ${OBJECTDIR}/ReadADC0.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/ReadADC0.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/ReadADC0.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/smc.o: smc.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/smc.o.d 
	@${RM} ${OBJECTDIR}/smc.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  smc.s  -o ${OBJECTDIR}/smc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/smc.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/smc.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/trig.o: trig.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/trig.o.d 
	@${RM} ${OBJECTDIR}/trig.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  trig.s  -o ${OBJECTDIR}/trig.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/trig.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/trig.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/SVGEN.o: SVGEN.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/SVGEN.o.d 
	@${RM} ${OBJECTDIR}/SVGEN.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  SVGEN.S  -o ${OBJECTDIR}/SVGEN.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/SVGEN.o.d"  -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/SVGEN.o.asm.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_REAL_ICE=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/SVGEN.o.d" "${OBJECTDIR}/SVGEN.o.asm.d"  -t $(SILENT)  -rsi ${MP_CC_DIR}../  
	
else
${OBJECTDIR}/SVGEN.o: SVGEN.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/SVGEN.o.d 
	@${RM} ${OBJECTDIR}/SVGEN.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  SVGEN.S  -o ${OBJECTDIR}/SVGEN.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/SVGEN.o.d"  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,-MD,"${OBJECTDIR}/SVGEN.o.asm.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/SVGEN.o.d" "${OBJECTDIR}/SVGEN.o.asm.d"  -t $(SILENT)  -rsi ${MP_CC_DIR}../  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/Lab2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/Lab2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG=__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)   -mreserve=data@0x1000:0x101B -mreserve=data@0x101C:0x101D -mreserve=data@0x101E:0x101F -mreserve=data@0x1020:0x1021 -mreserve=data@0x1022:0x1023 -mreserve=data@0x1024:0x1027 -mreserve=data@0x1028:0x104F   -Wl,--local-stack,,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D__DEBUG=__DEBUG,--defsym=__MPLAB_DEBUGGER_REAL_ICE=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--library=q,--no-force-link,--smart-io,--report-mem,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST) 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/Lab2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/Lab2.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -Wl,--local-stack,,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--library=q,--no-force-link,--smart-io,--report-mem,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST) 
	${MP_CC_DIR}\\xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/Lab2.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf  
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
