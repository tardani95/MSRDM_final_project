function getTF( tfS , HT, counter, stamp)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

tfS.Header.Stamp = stamp;
tfS.Header.Seq=counter;
% 
% tfS.ChildFrameId = name;
tfS.Transform.Translation.X = HT(1,4);
tfS.Transform.Translation.Y = HT(2,4);
tfS.Transform.Translation.Z = HT(3,4);
% 
rotm = HT(1:3,1:3);
quatrot = rotm2quat(rotm);
tfS.Transform.Rotation.W = quatrot(1);
tfS.Transform.Rotation.X = quatrot(2);
tfS.Transform.Rotation.Y = quatrot(3);
tfS.Transform.Rotation.Z = quatrot(4);

end

