# Dr.Sina: 
# script that changes id for labelimg -> 0 
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import os,glob
folder_path = '/home/abdessalem/Bureau/change_index/txt'
for filename in glob.glob(os.path.join(folder_path, '*.txt')):
	with open(filename, 'r') as f:
	    text = f.readlines()
	    l = []
	    for line in text :
	    	txt = line.split()
	    	txt_id = txt[0]
	    	print(txt_id)
	    	txt[len(txt)-1] = txt[len(txt)-1] +"\n"
	    	txt[0] = '0'
	    	print(txt)
	    	s = " ".join((txt))
	    	l.append(s)
	    print(l)
	    print(filename)
	    
	file1 = open(filename, 'w')
	file1.writelines(l)
	file1.close()
    


