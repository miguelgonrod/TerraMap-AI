USE mysql;

CREATE TABLE IF NOT EXISTS `maps` (
  `map_id` int NOT NULL AUTO_INCREMENT,
  `map_name` varchar(255) NOT NULL,
  `username` varchar(255) NOT NULL,
  `description` varchar(255) NOT NULL,
  `download_id_pgm` varchar(255) NOT NULL,
  `download_id_yaml` varchar(255) NOT NULL,
  `created_at` datetime NOT NULL,
  `updated_at` datetime NOT NULL,
  PRIMARY KEY (`map_id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8 AUTO_INCREMENT=1;