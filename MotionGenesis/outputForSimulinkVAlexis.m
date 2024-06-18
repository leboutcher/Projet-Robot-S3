function [t,VAR,Output] = outputForSimulink
%===========================================================================
% File: outputForSimulink.m created Jun 17 2024 by MotionGenesis 5.9.
% Portions copyright (c) 2009-2019 Motion Genesis LLC.  Rights reserved.
% MotionGenesis Get Started (Home) Licensee: Universite de Sherbrooke. (until August 2024).
% Paid-up MotionGenesis Get Started (Home) licensees are granted the right
% to distribute this code for legal student-academic (non-professional) purposes only,
% provided this copyright notice appears in all copies and distributions.
%===========================================================================
% The software is provided "as is", without warranty of any kind, express or    
% implied, including but not limited to the warranties of merchantability or    
% fitness for a particular purpose. In no event shall the authors, contributors,
% or copyright holders be liable for any claim, damages or other liability,     
% whether in an action of contract, tort, or otherwise, arising from, out of, or
% in connection with the software or the use or other dealings in the software. 
%===========================================================================
eventDetectedByIntegratorTerminate1OrContinue0 = [];
Torque=0;
Ffb=0; Fff=0; Nb=0; Nf=0; qDDt=0; qWBDDt=0; qWFDDt=0; xDDt=0;


%-------------------------------+--------------------------+-------------------+-----------------
% Quantity                      | Value                    | Units             | Description
%-------------------------------|--------------------------|-------------------|-----------------
b                               =  0.3;                    % UNITS               Constant
g                               =  9.81;                   % m/s^2               Constant
L                               =  0.5;                    % m                   Constant
LR                              =  0.5;                    % m                   Constant
mB                              =  0.0018;                 % kg                  Constant
mF                              =  0.0018;                 % kg                  Constant
mP                              =  0.5;                    % kg                  Constant
mR                              =  3;                      % kg                  Constant
rayon                           =  0.04;                   % m                   Constant

q                               =  0;                      % deg                 Initial Value
qWB                             =  0;                      % deg                 Initial Value
qWF                             =  0;                      % deg                 Initial Value
x                               =  0;                      % m                   Initial Value
qDt                             =  0;                      % deg/s               Initial Value
qWBDt                           =  0;                      % deg/s               Initial Value
qWFDt                           =  0;                      % deg/s               Initial Value
xDt                             =  0;                      % m/s                 Initial Value

tInitial                        =  0.0;                    % second              Initial Time
tFinal                          =  10;                     % sec                 Final Time
tStep                           =  0.01;                   % second              Integration Step
printIntScreen                  =  1;                      % 0 or +integer       0 is NO screen output
printIntFile                    =  1;                      % 0 or +integer       0 is NO file   output
absError                        =  1.0E-10;                %                     Absolute Error
relError                        =  1.0E-08;                %                     Relative Error
%-------------------------------+--------------------------+-------------------+-----------------

% Unit conversions
DEGtoRAD = pi / 180.0;
RADtoDEG = 180.0 / pi;
q = q * DEGtoRAD;
qWB = qWB * DEGtoRAD;
qWF = qWF * DEGtoRAD;
qDt = qDt * DEGtoRAD;
qWBDt = qWBDt * DEGtoRAD;
qWFDt = qWFDt * DEGtoRAD;

VAR = SetMatrixFromNamedQuantities;
[t,VAR,Output] = IntegrateForwardOrBackward( tInitial, tFinal, tStep, absError, relError, VAR, printIntScreen, printIntFile );
OutputToScreenOrFile( [], 0, 0 );   % Close output files.


%===========================================================================
function sys = mdlDerivatives( t, VAR, uSimulink )
%===========================================================================
SetNamedQuantitiesFromMatrix( VAR );

% Quantities to be specified (not assigned in MotionGenesis).
Torque = 0;
COEF = zeros( 8, 8 );
COEF(1,1) = 0.5*L*mP*cos(q);
COEF(1,2) = 1;
COEF(1,3) = 1;
COEF(1,6) = mB + mF + mP + mR;
COEF(2,1) = 0.5*L*mP*sin(q);
COEF(2,4) = -1;
COEF(2,5) = -1;
COEF(3,1) = 0.25*L*mP*(1.333333333333333*L+LR*sin(q)-2*rayon*cos(q));
COEF(3,5) = -LR;
COEF(3,6) = 0.5*L*mP*cos(q) - mB*rayon - mF*rayon - mP*rayon - mR*rayon;
COEF(3,7) = 0.5*mB*rayon^2;
COEF(3,8) = 0.5*mF*rayon^2;
COEF(4,1) = 0.3333333333333333*mP*L^2;
COEF(4,6) = 0.5*L*mP*cos(q);
COEF(5,2) = rayon;
COEF(5,7) = 0.5*mB*rayon^2;
COEF(6,3) = 2;
COEF(6,8) = mF*rayon;
COEF(7,6) = 1;
COEF(7,7) = rayon;
COEF(8,6) = 1;
COEF(8,8) = rayon;
RHS = zeros( 1, 8 );
RHS(1) = 0.5*L*mP*sin(q)*qDt^2;
RHS(2) = -g*mB - g*mF - g*mP - g*mR - 0.5*L*mP*cos(q)*qDt^2;
RHS(3) = -g*LR*mF - 0.5*g*LR*mP - 0.5*g*LR*mR - Torque - 0.5*g*L*mP*sin(q) - 0.5*L*mP*rayon*sin(q)*qDt^2 - 0.25*L*LR*mP*cos(q)*qDt^2;
RHS(4) = -0.5*g*L*mP*sin(q) - b*qDt;
RHS(5) = -Torque;
SolutionToAlgebraicEquations = COEF \ transpose(RHS);

% Update variables after uncoupling equations
qDDt = SolutionToAlgebraicEquations(1);
Ffb = SolutionToAlgebraicEquations(2);
Fff = SolutionToAlgebraicEquations(3);
Nb = SolutionToAlgebraicEquations(4);
Nf = SolutionToAlgebraicEquations(5);
xDDt = SolutionToAlgebraicEquations(6);
qWBDDt = SolutionToAlgebraicEquations(7);
qWFDDt = SolutionToAlgebraicEquations(8);

sys = transpose( SetMatrixOfDerivativesPriorToIntegrationStep );
end



%===========================================================================
function VAR = SetMatrixFromNamedQuantities
%===========================================================================
VAR = zeros( 1, 8 );
VAR(1) = q;
VAR(2) = qWB;
VAR(3) = qWF;
VAR(4) = x;
VAR(5) = qDt;
VAR(6) = qWBDt;
VAR(7) = qWFDt;
VAR(8) = xDt;
end


%===========================================================================
function SetNamedQuantitiesFromMatrix( VAR )
%===========================================================================
q = VAR(1);
qWB = VAR(2);
qWF = VAR(3);
x = VAR(4);
qDt = VAR(5);
qWBDt = VAR(6);
qWFDt = VAR(7);
xDt = VAR(8);
end


%===========================================================================
function VARp = SetMatrixOfDerivativesPriorToIntegrationStep
%===========================================================================
VARp = zeros( 1, 8 );
VARp(1) = qDt;
VARp(2) = qWBDt;
VARp(3) = qWFDt;
VARp(4) = xDt;
VARp(5) = qDDt;
VARp(6) = qWBDDt;
VARp(7) = qWFDDt;
VARp(8) = xDDt;
end



%===========================================================================
function Output = mdlOutputs( t, VAR, uSimulink )
%===========================================================================
Output = zeros( 1, 3 );
Output(1) = x;
Output(2) = q*RADtoDEG;
Output(3) = qWFDt*RADtoDEG;
end


%===========================================================================
function OutputToScreenOrFile( Output, shouldPrintToScreen, shouldPrintToFile )
%===========================================================================
persistent FileIdentifier hasHeaderInformationBeenWritten;

if( isempty(Output) ),
   if( ~isempty(FileIdentifier) ),
      fclose( FileIdentifier(1) );
      clear FileIdentifier;
      fprintf( 1, '\n Output is in the file outputForSimulink.1\n' );
      fprintf( 1, '\n Note: To automate plotting, issue the command OutputPlot in MotionGenesis.\n' );
      fprintf( 1, '\n To load and plot columns 1 and 2 with a solid line and columns 1 and 3 with a dashed line, enter:\n' );
      fprintf( 1, '    someName = load( ''outputForSimulink.1'' );\n' );
      fprintf( 1, '    plot( someName(:,1), someName(:,2), ''-'', someName(:,1), someName(:,3), ''--'' )\n\n' );
   end
   clear hasHeaderInformationBeenWritten;
   return;
end

if( isempty(hasHeaderInformationBeenWritten) ),
   if( shouldPrintToScreen ),
      fprintf( 1,                '%%       x              q            qWF''\n' );
      fprintf( 1,                '%%      (m)           (deg)        (deg/sec)\n\n' );
   end
   if( shouldPrintToFile && isempty(FileIdentifier) ),
      FileIdentifier(1) = fopen('outputForSimulink.1', 'wt');   if( FileIdentifier(1) == -1 ), error('Error: unable to open file outputForSimulink.1'); end
      fprintf(FileIdentifier(1), '%% FILE: outputForSimulink.1\n%%\n' );
      fprintf(FileIdentifier(1), '%%       x              q            qWF''\n' );
      fprintf(FileIdentifier(1), '%%      (m)           (deg)        (deg/sec)\n\n' );
   end
   hasHeaderInformationBeenWritten = 1;
end

if( shouldPrintToScreen ), WriteNumericalData( 1,                 Output(1:3) );  end
if( shouldPrintToFile ),   WriteNumericalData( FileIdentifier(1), Output(1:3) );  end
end


%===========================================================================
function WriteNumericalData( fileIdentifier, Output )
%===========================================================================
numberOfOutputQuantities = length( Output );
if( numberOfOutputQuantities > 0 ),
   for( i = 1 : numberOfOutputQuantities ),
      fprintf( fileIdentifier, ' %- 14.6E', Output(i) );
   end
   fprintf( fileIdentifier, '\n' );
end
end



%===========================================================================
function [functionsToEvaluateForEvent, eventTerminatesIntegration1Otherwise0ToContinue, eventDirection_AscendingIs1_CrossingIs0_DescendingIsNegative1] = EventDetection( t, VAR, uSimulink )
%===========================================================================
% Detects when designated functions are zero or cross zero with positive or negative slope.
% Step 1: Uncomment call to mdlDerivatives and mdlOutputs.
% Step 2: Change functionsToEvaluateForEvent,                      e.g., change  []  to  [t - 5.67]  to stop at t = 5.67.
% Step 3: Change eventTerminatesIntegration1Otherwise0ToContinue,  e.g., change  []  to  [1]  to stop integrating.
% Step 4: Change eventDirection_AscendingIs1_CrossingIs0_DescendingIsNegative1,  e.g., change  []  to  [1].
% Step 5: Possibly modify function EventDetectedByIntegrator (if eventTerminatesIntegration1Otherwise0ToContinue is 0).
%---------------------------------------------------------------------------
% mdlDerivatives( t, VAR, uSimulink );        % UNCOMMENT FOR EVENT HANDLING
% mdlOutputs(     t, VAR, uSimulink );        % UNCOMMENT FOR EVENT HANDLING
functionsToEvaluateForEvent = [];
eventTerminatesIntegration1Otherwise0ToContinue = [];
eventDirection_AscendingIs1_CrossingIs0_DescendingIsNegative1 = [];
eventDetectedByIntegratorTerminate1OrContinue0 = eventTerminatesIntegration1Otherwise0ToContinue;
end


%===========================================================================
function [isIntegrationFinished, VAR] = EventDetectedByIntegrator( t, VAR, nIndexOfEvents )
%===========================================================================
isIntegrationFinished = eventDetectedByIntegratorTerminate1OrContinue0( nIndexOfEvents );
if( ~isIntegrationFinished ),
   SetNamedQuantitiesFromMatrix( VAR );
%  Put code here to modify how integration continues.
   VAR = SetMatrixFromNamedQuantities;
end
end



%===========================================================================
function [t,VAR,Output] = IntegrateForwardOrBackward( tInitial, tFinal, tStep, absError, relError, VAR, printIntScreen, printIntFile )
%===========================================================================
OdeMatlabOptions = odeset( 'RelTol',relError, 'AbsTol',absError, 'MaxStep',tStep, 'Events',@EventDetection );
t = tInitial;                 epsilonT = 0.001*tStep;                   tFinalMinusEpsilonT = tFinal - epsilonT;
printCounterScreen = 0;       integrateForward = tFinal >= tInitial;    tAtEndOfIntegrationStep = t + tStep;
printCounterFile   = 0;       isIntegrationFinished = 0;
mdlDerivatives( t, VAR, 0 );
while 1,
   if( (integrateForward && t >= tFinalMinusEpsilonT) || (~integrateForward && t <= tFinalMinusEpsilonT) ), isIntegrationFinished = 1;  end
   shouldPrintToScreen = printIntScreen && ( isIntegrationFinished || printCounterScreen <= 0.01 );
   shouldPrintToFile   = printIntFile   && ( isIntegrationFinished || printCounterFile   <= 0.01 );
   if( isIntegrationFinished || shouldPrintToScreen || shouldPrintToFile ),
      Output = mdlOutputs( t, VAR, 0 );
      OutputToScreenOrFile( Output, shouldPrintToScreen, shouldPrintToFile );
      if( isIntegrationFinished ), break;  end
      if( shouldPrintToScreen ), printCounterScreen = printIntScreen;  end
      if( shouldPrintToFile ),   printCounterFile   = printIntFile;    end
   end
   [TimeOdeArray, VarOdeArray, timeEventOccurredInIntegrationStep, nStatesArraysAtEvent, nIndexOfEvents] = ode45( @mdlDerivatives, [t tAtEndOfIntegrationStep], VAR, OdeMatlabOptions, 0 );
   if( isempty(timeEventOccurredInIntegrationStep) ),
      lastIndex = length( TimeOdeArray );
      t = TimeOdeArray( lastIndex );
      VAR = VarOdeArray( lastIndex, : );
      printCounterScreen = printCounterScreen - 1;
      printCounterFile   = printCounterFile   - 1;
      if( abs(tAtEndOfIntegrationStep - t) >= abs(epsilonT) ), warning('numerical integration failed'); break;  end
      tAtEndOfIntegrationStep = t + tStep;
      if( (integrateForward && tAtEndOfIntegrationStep > tFinal) || (~integrateForward && tAtEndOfIntegrationStep < tFinal) ) tAtEndOfIntegrationStep = tFinal;  end
   else
      t = timeEventOccurredInIntegrationStep( 1 );    % time  at firstEvent = 1 during this integration step.
      VAR = nStatesArraysAtEvent( 1, : );             % state at firstEvent = 1 during this integration step.
      printCounterScreen = 0;
      printCounterFile   = 0;
      [isIntegrationFinished, VAR] = EventDetectedByIntegrator( t, VAR, nIndexOfEvents(1) );
   end
end
end


%==========================================
end    % End of function outputForSimulink
%==========================================
