var Engine = require('tingodb')(),
    assert = require('assert');
var Cursor = require('tingodb').Cursor;
var db = new Engine.Db('/Users/carlo/ESP/mine/Q4P26', {});
var collection = db.collection("batch_document_insert_collection_safe");
var fs = require('fs');

var array = fs.readFileSync('smoke.txt', 'utf8').toString().split('\r\n');
for(let line of array){
  var theline = line.split('\t');
  collection.insert({"Time":theline[0], "Id":theline[1], "Smoke":theline[2], "Temp":theline[3]}
    , function(err, result) {
    assert.equal(null, err);
  });
}

console.log("printing temperatures for ID = 1:");
var cursor = db.collection('batch_document_insert_collection_safe').find({"Id": "1"});
cursor.each(function(err, doc) {
  if(doc != null) {
    console.log(doc.Temp)
  }
});
