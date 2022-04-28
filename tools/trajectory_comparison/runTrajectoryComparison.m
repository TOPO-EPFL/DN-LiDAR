addpath('functions');

clear config

config.diffwrtref = true;
config.reference = 1;
config.orient_binwidth = [0.01, 0.01, 0.02];
config.pos_binwidth = [0.002, 0.002, 0.002];


config.t0 = 396400;
config.tend = 397000;

config.trjs = {};

config.trjs{end+1}.type = 'posproc';
config.trjs{end}.name = 'T.0 R';
config.trjs{end}.path = '../../data/trajectories/T0-R.out';
config.trjs{end}.lat0 = 46.566641300900;
config.trjs{end}.lon0 = 6.512265144377;

config.trjs{end+1}.type = 'posproc';
config.trjs{end}.name = 'T.1 KS';
config.trjs{end}.path = '../../data/trajectories/T1-KS.out';
config.trjs{end}.lat0 = 46.566641300900;
config.trjs{end}.lon0 = 6.512265144377;

config.trjs{end+1}.type = 'roamfree';
config.trjs{end}.name = 'T.2 DNC';
config.trjs{end}.path = '../../data/trajecotries/T2-DNC.tar.gz';

compareTrajectories(config);