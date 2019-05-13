var express = require("express");
var app     = express();
var path    = require("path");
var os = require( 'os' );
var formidable = require('formidable');
var fs = require('fs');
const spawn = require("child_process").spawn;
app.use(express.static(__dirname + '/pages' ));
const bodyParser = require('body-parser')
var ip = require("ip");
app.use(bodyParser.json())
app.use(bodyParser.urlencoded({ extended: true }))

app.get('/',function(req,res){
  res.sendFile(path.join(__dirname+'/main.html'));

});
app.get('/ip',function(req,res){
    res.send(ip.address())
	console.log(ip.address())
});

var newpath;
app.post('/fileupload',function(req,res){

 var form = new formidable.IncomingForm();
    form.parse(req, function (err, fields, files) {
      var oldpath = files.filetoupload.path;
      newpath = files.filetoupload.name;
      fs.rename(oldpath, newpath, function (err) {
        if (err) throw err;
        res.redirect('/');
		var option2="name="+"'"+newpath+"';runThis;exit;"
		var m = spawn('matlab', ['-r', option2])
      });
});
});

app.get('/download', function(req, res) {

  res.download(__dirname+'/ModelingResults.zip')
  //fs.unlinkSync(__dirname+'/ModelingResults.zip')
});

app.post('/option', function(req, res){
console.log(req.body.value)
console.log(newpath)
var option2="x="+"'"+req.body.value+"',"+"name="+"'"+newpath+"';runThis;"//exit;
var m = spawn('matlab', ['-r', option2])
res.send('Got response')
});

app.listen(8080);

console.log("Running at Port 8080");
