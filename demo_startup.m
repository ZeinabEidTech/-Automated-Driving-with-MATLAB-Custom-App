% demo_startup.m — Launch the Automated Driving App
try
    app = AutomatedDrivingApp;
    app.run();
catch ME
    fprintf(2, 'Error launching app: %s\n', ME.message);
end