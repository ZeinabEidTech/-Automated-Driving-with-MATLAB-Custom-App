classdef AutomatedDrivingApp < handle
    % AutomatedDrivingApp - Lightweight MATLAB app for lane & vehicle detection
    % Zainab Eid - Custom code for "Automated Driving with MATLAB"
    %
    % Requirements (auto-detected at runtime where possible):
    % - MATLAB R2021b or later (recommended)
    % - Computer Vision Toolbox
    % - Automated Driving Toolbox (optional for vehicleDetectorACF, BEV)
    %
    % Usage:
    %   app = AutomatedDrivingApp;
    %   app.run();
    
    properties
        UIFig           matlab.ui.Figure
        UIAxes          matlab.ui.control.UIAxes
        LoadBtn         matlab.ui.control.Button
        PlayBtn         matlab.ui.control.Button
        StopBtn         matlab.ui.control.Button
        SnapshotBtn     matlab.ui.control.Button
        StatusLabel     matlab.ui.control.Label
        ThresholdSlider matlab.ui.control.Slider
        ThresholdLbl    matlab.ui.control.Label
        VideoReader     % video reader object
        VideoFile       char
        IsPlaying       logical = false
        LaneParams      struct
        Detector        % vehicle detector
        FrameIdx        double = 0
        FPSLabel        matlab.ui.control.Label
        InfoText        matlab.ui.control.TextArea
    end
    
    methods
        function self = AutomatedDrivingApp()
            self.buildUI();
            self.LaneParams = defaultLaneParams();
            self.Detector = initVehicleDetector();
        end
        
        function run(self)
            movegui(self.UIFig, 'center');
            self.UIFig.Visible = 'on';
        end
    end
    
    methods (Access = private)
        function buildUI(self)
            self.UIFig = uifigure('Name','Automated Driving Demo - Zainab Eid', 'Position',[100 100 1200 720]);
            g = uigridlayout(self.UIFig,[6, 6]);
            g.RowHeight = {30, 30, '1x', 30, 120, 30};
            g.ColumnWidth = {140, 140, 140, '1x', 160, 160};
            
            % Row 1: Buttons
            self.LoadBtn = uibutton(g, 'Text','Load Video', 'ButtonPushedFcn', @(~,~)self.loadVideo());
            self.LoadBtn.Layout.Row = 1; self.LoadBtn.Layout.Column = 1;
            
            self.PlayBtn = uibutton(g, 'Text','▶ Play', 'ButtonPushedFcn', @(~,~)self.onPlay());
            self.PlayBtn.Layout.Row = 1; self.PlayBtn.Layout.Column = 2;
            
            self.StopBtn = uibutton(g, 'Text','⏹ Stop', 'ButtonPushedFcn', @(~,~)self.onStop());
            self.StopBtn.Layout.Row = 1; self.StopBtn.Layout.Column = 3;
            
            self.SnapshotBtn = uibutton(g, 'Text','📸 Snapshot', 'ButtonPushedFcn', @(~,~)self.onSnapshot());
            self.SnapshotBtn.Layout.Row = 1; self.SnapshotBtn.Layout.Column = 5;
            
            self.FPSLabel = uilabel(g, 'Text','FPS: -');
            self.FPSLabel.Layout.Row = 1; self.FPSLabel.Layout.Column = 6;
            
            % Row 2: Threshold slider
            self.ThresholdLbl = uilabel(g, 'Text','Lane Edge Sensitivity');
            self.ThresholdLbl.Layout.Row = 2; self.ThresholdLbl.Layout.Column = 1;
            self.ThresholdSlider = uislider(g, 'Limits',[0.05 0.5], 'Value',0.2, 'ValueChangedFcn', @(~,~)self.onParamChanged());
            self.ThresholdSlider.Layout.Row = 2; self.ThresholdSlider.Layout.Column = [2 3];
            
            % Row 3: Axes
            self.UIAxes = uiaxes(g); self.UIAxes.Layout.Row = 3; self.UIAxes.Layout.Column = [1 6];
            title(self.UIAxes, 'Video Frame with Lanes & Vehicles');
            xlabel(self.UIAxes, 'X'); ylabel(self.UIAxes, 'Y');
            
            % Row 4: Status
            self.StatusLabel = uilabel(g, 'Text','Load a video to start.');
            self.StatusLabel.Layout.Row = 4; self.StatusLabel.Layout.Column = [1 6];
            
            % Row 5: Info box
            self.InfoText = uitextarea(g,'Editable','off');
            self.InfoText.Layout.Row = 5; self.InfoText.Layout.Column = [1 6];
            self.InfoText.Value = ["Welcome to the Automated Driving demo."
                                   "Load an MP4/AVI video, press Play."
                                   "This app detects lanes (Hough) & vehicles (ACF or cascade fallback)."
                                   "Adjust edge sensitivity if lanes are noisy."];
            
            % Row 6: Footer
            uilabel(g, 'Text','© 2025 Zainab Eid — MATLAB Automated Driving Prototype','HorizontalAlignment','center').Layout.Row = 6;
            ans = g; %#ok<NASGU>
        end
        
        function onParamChanged(self)
            self.LaneParams.edgeThreshold = self.ThresholdSlider.Value;
        end
        
        function loadVideo(self)
            try
                [f,p] = uigetfile({'*.mp4;*.avi','Video Files (*.mp4, *.avi)'}, 'Select a driving video');
                if isequal(f,0)
                    return;
                end
                self.VideoFile = fullfile(p,f);
                self.VideoReader = VideoReader(self.VideoFile);
                self.FrameIdx = 0;
                self.printInfo(sprintf("Loaded: %s (%dx%d @ %.2f fps)", f, self.VideoReader.Width, self.VideoReader.Height, self.VideoReader.FrameRate));
                self.StatusLabel.Text = 'Video loaded. Press Play.';
                frame = readFrame(self.VideoReader);
                self.FrameIdx = 1;
                imshow(frame, 'Parent', self.UIAxes);
            catch ME
                uialert(self.UIFig, ME.message, 'Load Error');
            end
        end
        
        function onPlay(self)
            if isempty(self.VideoReader)
                self.StatusLabel.Text = 'Please load a video first.';
                return;
            end
            self.IsPlaying = true;
            self.StatusLabel.Text = 'Playing...';
            lastTic = tic;
            framesCount = 0;
            while self.IsPlaying && hasFrame(self.VideoReader)
                frame = readFrame(self.VideoReader);
                self.FrameIdx = self.FrameIdx + 1;
                [lanedImg, lanesInfo] = detectLanes(frame, self.LaneParams);
                [detImg, bboxes, scores] = detectVehicles(lanedImg, self.Detector);
                annotated = insertVehicleInfo(detImg, bboxes, scores);
                imshow(annotated, 'Parent', self.UIAxes);
                self.StatusLabel.Text = sprintf('Frame %d — vehicles: %d — lanes: %s', ...
                    self.FrameIdx, size(bboxes,1), lanesInfo.summary);
                drawnow limitrate;
                framesCount = framesCount + 1;
                if toc(lastTic) >= 1.0
                    self.FPSLabel.Text = sprintf('FPS: %d', framesCount);
                    framesCount = 0; lastTic = tic;
                end
            end
        end
        
        function onStop(self)
            self.IsPlaying = false;
            self.StatusLabel.Text = 'Stopped.';
        end
        
        function onSnapshot(self)
            if isempty(self.VideoReader)
                self.StatusLabel.Text = 'Load a video first.'; return;
            end
            frame = getframe(self.UIAxes);
            ts = datestr(now,'yyyymmdd_HHMMSS');
            fname = sprintf('snapshot_%s.png', ts);
            imwrite(frame.cdata, fname);
            self.printInfo("Saved snapshot: " + string(fname));
            uialert(self.UIFig, "Snapshot saved: " + string(fname), 'Saved');
        end
        
        function printInfo(self, msg)
            self.InfoText.Value = [self.InfoText.Value; string(msg)];
            self.InfoText.Value = self.InfoText.Value(max(1,end-10):end);
        end
    end
end

function params = defaultLaneParams()
params = struct();
params.edgeThreshold = 0.2; % Canny threshold (scaled)
params.minLength = 30;      % Hough line minimum length (pixels)
params.fillGap = 20;        % Hough line fill gap
end

function [outImg, info] = detectLanes(frame, params)
% Simple lane detection using grayscale + Canny + Hough
if ~isa(frame,'uint8'); frame = im2uint8(frame); end
gray = rgb2gray(frame);
edges = edge(gray, 'Canny', params.edgeThreshold);
[H, theta, rho] = hough(edges);
peaks = houghpeaks(H, 8, 'Threshold', ceil(0.3*max(H(:))));
linesStruct = houghlines(edges, theta, rho, peaks, 'FillGap', params.fillGap, 'MinLength', params.minLength);

outImg = frame;
info.summary = "none";
if ~isempty(linesStruct)
    outImg = insertShape(outImg, 'Line', reshape([[linesStruct.point1]' [linesStruct.point2]']',1,[]) , 'LineWidth', 4);
    info.summary = sprintf('%d lines', numel(linesStruct));
end
end

function detector = initVehicleDetector()
% Try ACF detector (Automated Driving Toolbox), otherwise cascade fallback
detector = [];
try
    detector = vehicleDetectorACF();
catch
    try
        detector = vision.CascadeObjectDetector('Car');
    catch
        detector = [];
    end
end
end

function [outImg, bboxes, scores] = detectVehicles(frame, detector)
outImg = frame; bboxes = zeros(0,4); scores = [];
if isempty(detector); return; end
try
    [bboxes, scores] = detect(detector, frame);
catch
    % Cascade API
    try
        bboxes = step(detector, frame);
        scores = ones(size(bboxes,1),1);
    catch
        bboxes = zeros(0,4); scores = [];
    end
end
if ~isempty(bboxes)
    outImg = insertObjectAnnotation(frame, 'rectangle', bboxes, arrayfun(@(s)sprintf('%.2f',s), scores, 'UniformOutput', false));
end
end

function annotated = insertVehicleInfo(img, bboxes, scores)
annotated = img;
if isempty(bboxes); return; end
% Safety heuristic: if box width > threshold => too close
thresh = max(40, round(size(img,2)*0.08));
warn = any(bboxes(:,3) > thresh);
if warn
    annotated = insertText(annotated, [10 10], 'WARNING: Vehicle too close — slow down', 'FontSize', 16, 'BoxOpacity',0.7);
else
    annotated = insertText(annotated, [10 10], 'Safe distance', 'FontSize', 16, 'BoxOpacity',0.7);
end
end