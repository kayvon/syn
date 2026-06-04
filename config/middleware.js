exports.setSettings = function(app, express){
  var io = app.get('io');
  app.set('views', __dirname + '/../views');
  app.set("view engine", "jade");
  app.use(require('stylus').middleware({ src: __dirname + '/../public'}));
  app.use(express.static(__dirname + '/../public/lib'));
  app.use(express.static(__dirname + '/../public'));
};
