//Accuracy (spheres and cylinders)
$fn=50;

//Outside design of the box
module external(length, width, height)
{
	corners_rad=height/10.;
	length=length-2*corners_rad;
	width=width-2*corners_rad;
	height=height-2*corners_rad;


	translate([corners_rad,corners_rad,corners_rad])
	{	
		//Rounded box
		hull()
		{
			cube([length, width, height]);
		
			sphere(corners_rad);
			translate([length,0,0]){sphere(corners_rad);}
			translate([length,width,0]){sphere(corners_rad);}
			translate([0,width,0]){sphere(corners_rad);}
			translate([length,width,height]){sphere(corners_rad);}
			translate([0,width,height]){sphere(corners_rad);}
			translate([0,0,height]){sphere(corners_rad);}
			translate([length,0,height]){sphere(corners_rad);}
		}
	}

	//shoulder_stand(length, width, height, corners_rad);
}

//Internal design of the box
module internal(length, width, height, thickness)
{
	translate([thickness, thickness, thickness])
	{
		cube([length-2*thickness, width-2*thickness, height-2*thickness]);
	}
}

//Shoulder stand
module shoulder_stand(length, width, height, corners_rad)
{
	//Shoulder "stand"
	radius=100;
	translate([corners_rad,width/2.,-radius+3]){
	rotate([0,90,0]){
		difference()
		{
			cylinder(h=length, r1=radius, r2=radius, center=false);
			translate([5,0,-length/2.]){
				cylinder(h=2*length, r1=radius, r2=radius, center=false);
			}
			translate([-radius+15,-1.5*radius,-length/2.]){
				cube([radius,3*radius,2*length]);
			}
		}
	}}
}



module thebox(length, width, height, thickness, cover_thickness)
{
	sewing_holes_pos=[[length*0.2, width/4.5, 0], [length*0.2, width*3.5/4.5, 0],[length*0.8, width/4.5, 0],[length*0.8, width*3.5/4.5, 0]];

	difference()
	{
		external(length, width, height);
		internal(length, width, height, thickness);
		//Screw holes
		translate([2, 2, height]){cylinder(h=1.5*thickness, r1=1, r2=1, center = true);/*head*/cylinder(h=1*thickness, r1=1.5, r2=1.5, center = true);}
		translate([2, width-2, height]){cylinder(h=1.5*thickness, r1=1, r2=1, center = true);/*head*/cylinder(h=1*thickness, r1=1.5, r2=1.5, center = true);}
		translate([length-2, width-2, height]){cylinder(h=1.5*thickness, r1=1, r2=1, center = true);/*head*/cylinder(h=1*thickness, r1=1.5, r2=1.5, center = true);}
		translate([length-2, 2, height]){cylinder(h=1.5*thickness, r1=1, r2=1, center = true);/*head*/cylinder(h=1*thickness, r1=1.5, r2=1.5, center = true);}
		

		//Switches
			//Power
			translate([thickness+6.5, (width-19)/2., height-thickness]){rotate([0,0,90]){cube([19, 6.5, 1.5*thickness]);}}
			/*//Mode
			translate([length-thickness, (width-16)/2., 0.6*height]){cube([19, 16, 1.5*thickness]);}*/

		//Sound holes
		translate([length*0.3, width/2., height-thickness/2.]){
			translate([-1, 0, 0]){
			cylinder(h=1.5*thickness, r1=0.6, r2=0.6, center = true);}
			translate([1, 0, 0]){
			cylinder(h=1.5*thickness, r1=0.6, r2=0.6, center = true);}
			translate([0, 1, 0]){
			cylinder(h=1.5*thickness, r1=0.6, r2=0.6, center = true);}
			translate([0, -1, 0]){
			cylinder(h=1.5*thickness, r1=0.6, r2=0.6, center = true);}
		}

		//Sewing holes (buttons like)
		//Four 1mm holes for each, 4mm apparts
		for(i=sewing_holes_pos)
		{
			translate(i)
				translate([2, 0, 0])
				cylinder(h=2.5*thickness, r1=0.5, r2=0.5, center = true);
		}
		for(i=sewing_holes_pos)
		{
			translate(i)
				translate([-2, 0, 0])
				cylinder(h=2.5*thickness, r1=0.5, r2=0.5, center = true);
		}
		for(i=sewing_holes_pos)
		{
			translate(i)
				translate([0, 2, 0])
				cylinder(h=2.5*thickness, r1=0.5, r2=0.5, center = true);
		}
		for(i=sewing_holes_pos)
		{
			translate(i)
				translate([0, -2, 0])
				cylinder(h=2.5*thickness, r1=0.5, r2=0.5, center = true);
		}


		//Power cables: 4.5cm appart
		rotate([90,0,0]){translate([(length-45)/2., 5, 0]){cylinder(h=2.5*thickness, r1=1, r2=1, center = true);}}
		rotate([90,0,0]){translate([(length-45)/2., 5, -width]){cylinder(h=2.5*thickness, r1=1, r2=1, center = true);}}
		rotate([90,0,0]){translate([(length-45)/2.+45, 5, 0]){cylinder(h=2.5*thickness, r1=1, r2=1, center = true);}}
		rotate([90,0,0]){translate([(length-45)/2.+45, 5, -width]){cylinder(h=2.5*thickness, r1=1, r2=1, center = true);}}
	}
}





//scale([0.001,0.001,0.001])
{
	cover_thickness=5;
	thickness=3;	

	//External dim
	length=63+2*thickness;
	width=20+2*thickness;
	height=22+2*thickness;

	
	/*//The botom
	difference()
	{
		thebox(length, width, height, thickness, cover_thickness);
		//remove top
		translate([-5, -5, height-cover_thickness]){
		cube([length+10, width+10, cover_thickness+10]);}
	}*/

	translate([0,0,2*cover_thickness])
	{
		//Box cover
		difference()
		{
			thebox(length, width, height, thickness, cover_thickness);
			//remove bottom
			translate([-5, -5, -5]){
			cube([length+10, width+10, height-cover_thickness+5]);}
		}
	}
}